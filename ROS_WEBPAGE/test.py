from flask import Flask, Response, render_template, jsonify, request, send_file
import cv2, threading, time, io, numpy as np, serial, platform
import matplotlib
matplotlib.use('Agg') # Set non-interactive backend before importing pyplot
import matplotlib.pyplot as plt
from datetime import datetime
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor

# ============================================================
# Flask setup
# ============================================================
app = Flask(__name__)
app.secret_key = "team_automatons_secret_key"

# Global state
# ============================================================
latest_data = {"latitude": "N/A", "longitude": "N/A"}
target_resolutions = {} # Stores desired resolution per camera index: {0: (160, 110, 8), ...}
saved_coords = [
    {"latitude": "19.000000", "longitude": "72.000000", "ts": "10:00:00", "color": "Yellow", "type": "Pickup", "desc": "Start Point"},
    {"latitude": "19.000000", "longitude": "72.001000", "ts": "10:00:05", "color": "Blue", "type": "Delivery", "desc": "Checkpoint 1"},
    {"latitude": "19.001000", "longitude": "72.001000", "ts": "10:00:10", "color": "Red", "type": "Pickup", "desc": "Target A"},
    {"latitude": "19.001000", "longitude": "72.000000", "ts": "10:00:15", "color": "Yellow", "type": "Delivery", "desc": "Dropoff Z"},
    {"latitude": "19.000453", "longitude": "72.000035", "ts": "10:00:20", "color": "Blue", "type": "Pickup", "desc": "End Zone"},
]

# ... existing code ...

@app.route("/store_coord", methods=["POST"])
def store_coord():
    d = request.json
    if not d:
        return jsonify({"error": "No JSON data"}), 400
    saved_coords.append({
        "latitude": d.get("latitude", "N/A"),
        "longitude": d.get("longitude", "N/A"),
        "ts": datetime.now().strftime("%H:%M:%S"),
        "color": d.get("color", "Unknown"),
        "type": d.get("type", "Unknown"),
        "desc": d.get("desc", "")
    })
    return jsonify({"ok": True})

@app.route("/clear_coords", methods=["POST"])
def clear_coords():
    saved_coords.clear()
    return jsonify({"ok": True})


@app.route("/path_data")
def path_data():
    """Returns the full list of coordinates for the interactive map."""
    return jsonify(saved_coords)

# ... remaining code ...

astrobio_sensors = {
    "NPK": "80 mg/kg",
    "AHT21": "25°C / 58%",
    "MQ2": "Clean air",
    "MQ7": "Low CO",
    "BMP180": "1012 hPa"
}

# ============================================================
# Panorama state
# ============================================================
panorama_frames = []
panorama_lock = threading.Lock()

# ============================================================
# OpenCV Webcams - Improved Thread-Safe Design
# ============================================================
opencv_frames = {}          # camera_id -> latest frame (numpy array)
opencv_caps = {}            # camera_id -> VideoCapture object
opencv_status = {}          # camera_id -> status dict
frame_locks = {}            # camera_id -> individual lock (reduces contention)
camera_stop_events = {}     # camera_id -> threading.Event (for graceful shutdown)
opencv_global_lock = threading.Lock()  # Only for adding/removing cameras


def get_frame_lock(idx):
    """Get or create a per-camera lock to reduce contention"""
    with opencv_global_lock:
        if idx not in frame_locks:
            frame_locks[idx] = threading.Lock()
        return frame_locks[idx]


def is_black_frame(frame, threshold=5):
    """
    Check if a frame is essentially black (camera not producing real frames).
    Returns True if frame is black/nearly black.
    """
    if frame is None or frame.size == 0:
        return True
    # Convert to grayscale if color
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame
    return np.mean(gray) < threshold


def reinit_camera(idx, configs=None):
    """
    Reinitialize a single camera with multiple configuration attempts.
    Returns (cap, config) tuple or (None, None) on failure.
    
    IMPROVED: Validates frames are not black and tries multiple backends.
    """
    if configs is None:
        configs = [
            # (640, 480, 15),   # Higher resolution first
            # (640, 480, 10),
            # (320, 240, 15),
            # (320, 240, 10),   # Lower settings as fallback
            (160,110,8),
        ]
    
    # Try multiple backends on Windows (DSHOW can be problematic)
    if platform.system() == "Windows":
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF]
    else:
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
    
    for backend in backends:
        try:
            cap = cv2.VideoCapture(idx, backend)
        except Exception as e:
            print(f"⚠️ Camera {idx} backend {backend} failed: {e}")
            continue
        
        if not cap.isOpened():
            try:
                cap.release()
            except:
                pass
            continue
        
        for width, height, fps in configs:
            try:
                # Set MJPG format (compressed, uses less USB bandwidth)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Give camera time to apply settings
                time.sleep(0.5)
                
                # Flush buffer - read and discard a few frames
                for _ in range(3):
                    cap.read()
                    time.sleep(0.05)
                
                # Try to read valid frames (not black)
                valid_frames = 0
                for attempt in range(10):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        if not is_black_frame(frame):
                            valid_frames += 1
                            if valid_frames >= 2:  # Need 2 good frames to confirm
                                backend_name = "DSHOW" if backend == cv2.CAP_DSHOW else "MSMF" if backend == cv2.CAP_MSMF else "V4L2"
                                print(f"✅ Camera {idx} working with {backend_name} at {width}x{height}@{fps}")
                                return cap, (width, height, fps)
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"⚠️ Camera {idx} config {width}x{height}@{fps} failed: {e}")
                continue
        
        # This backend didn't work, release and try next
        try:
            cap.release()
        except:
            pass
    
    # All backends and configs failed
    print(f"❌ Camera {idx} could not be initialized with any backend/config")
    return None, None


def webcam_reader(idx, cap, stop_event):
    """
    Read frames from webcam with crash recovery and auto-reconnect.
    Uses per-camera lock for thread safety.
    
    FIXED: Proper loop structure with separate main loop and reconnect logic.
    """
    consecutive_failures = 0
    max_failures = 30
    reconnect_attempts = 0
    max_reconnects = 5
    current_cap = cap
    frame_lock = get_frame_lock(idx)
    current_config = None # Track current resolution
    
    # Update status
    with opencv_global_lock:
        opencv_status[idx] = {"state": "running", "reconnects": 0}
    
    while not stop_event.is_set():
        try:
            # Check if camera is valid
            if current_cap is None or not current_cap.isOpened():
                raise Exception("Camera not available")
            
            # Check for dynamic resolution change
            if idx in target_resolutions:
                desired = target_resolutions[idx]
                if current_config != desired:
                    print(f"🔄 Applying new resolution for Camera {idx}: {desired}")
                    if current_cap:
                        current_cap.release()
                    
                    # Try to apply new config
                    current_cap, config = reinit_camera(idx, [desired])
                    
                    if current_cap is None:
                        print(f"⚠️ Failed to switch to {desired}. Removing invalid target and reverting.")
                        # Remove invalid target to prevent infinite loop
                        try:
                            del target_resolutions[idx]
                        except:
                            pass
                        raise Exception("Failed to re-init with new resolution")
                    
                    current_config = config
                    # Continue loop to read frame with new settings immediately
                    continue 

            # Read frame with implicit timeout (DSHOW has internal timeout)
            ret, frame = current_cap.read()
            
            if ret and frame is not None and frame.size > 0:
                # Success - update frame and reset failure counters
                consecutive_failures = 0
                reconnect_attempts = 0
                
                with frame_lock:
                    opencv_frames[idx] = frame.copy()
                
                # Small throttle to reduce CPU usage
                time.sleep(0.01)
            else:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    raise Exception(f"Too many read failures ({consecutive_failures})")
                time.sleep(0.05)
                
        except Exception as e:
            if stop_event.is_set():
                break
                
            print(f"⚠️ Camera {idx} error: {e} - attempting reconnect...")
            consecutive_failures = 0
            reconnect_attempts += 1
            
            # Update status
            with opencv_global_lock:
                opencv_status[idx] = {"state": "reconnecting", "reconnects": reconnect_attempts}
            
            # Release old capture safely
            try:
                if current_cap is not None:
                    current_cap.release()
            except:
                pass
            current_cap = None
            
            # Check if we've exhausted reconnect attempts
            if reconnect_attempts >= max_reconnects:
                print(f"🛑 Camera {idx} permanently stopped after {max_reconnects} reconnect attempts")
                break
            
            # Exponential backoff before reconnecting
            wait_time = min(2 ** reconnect_attempts, 10)
            
            # Wait with periodic stop_event checks
            for _ in range(int(wait_time * 10)):
                if stop_event.is_set():
                    break
                time.sleep(0.1)
            
            if stop_event.is_set():
                break
            
            # Try to reinitialize
            current_cap, config = reinit_camera(idx)
            current_config = config
            if current_cap is not None:
                print(f"✅ Camera {idx} reconnected successfully with config {config}")
                with opencv_global_lock:
                    opencv_caps[idx] = current_cap
                    opencv_status[idx] = {"state": "running", "reconnects": reconnect_attempts}
            else:
                print(f"❌ Camera {idx} reconnect failed (attempt {reconnect_attempts}/{max_reconnects})")
    
    # Cleanup on exit
    print(f"🔴 Camera {idx} reader thread stopping")
    with opencv_global_lock:
        if idx in opencv_caps:
            try:
                opencv_caps[idx].release()
            except:
                pass
            del opencv_caps[idx]
        opencv_status[idx] = {"state": "stopped", "reconnects": reconnect_attempts}


def init_single_camera(idx):
    """
    Initialize a single camera. Returns (idx, success, cap) tuple.
    Used for parallel initialization.
    """
    cap, config = reinit_camera(idx)
    if cap is not None:
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"✅ Webcam detected at index {idx} ({actual_w}x{actual_h})")
        return (idx, True, cap, config)
    return (idx, False, None, None)


def init_webcams(max_index=10):
    """
    Initialize webcams with proper USB hub support.
    
    IMPROVED: Better duplicate detection and validation that cameras produce real frames.
    Reduced max_index to 10 to speed up scanning.
    """
    print("🔍 Scanning for cameras...")
    
    # First, do a quick scan to find which indices have cameras
    potential_cameras = []
    backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_V4L2
    
    for i in range(max_index):
        try:
            cap = cv2.VideoCapture(i, backend)
            if cap.isOpened():
                # Quick validation: try to read a frame
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    # Check it's not a black frame
                    if not is_black_frame(frame):
                        potential_cameras.append(i)
                        print(f"  📹 Index {i}: Camera responds with valid frames")
                    else:
                        print(f"  ⚫ Index {i}: Camera only produces black frames - SKIPPING")
                else:
                    print(f"  ⚠️ Index {i}: Camera opens but cannot read frames - SKIPPING")
            cap.release()
        except Exception as e:
            print(f"  ❌ Index {i}: Error - {e}")
        time.sleep(0.1)  # Small delay between scans
    
    print(f"\n📷 Found {len(potential_cameras)} working camera(s) at indices: {potential_cameras}")
    
    if not potential_cameras:
        print("⚠️ No cameras detected!")
        return
    
    # Initialize cameras one at a time with proper delays
    detected_cameras = 0
    
    # Store reference frames for duplicate detection (multiple samples per camera)
    reference_data = {}  # idx -> list of frame samples
    
    for idx in potential_cameras:
        print(f"\n🔄 Initializing camera {idx}...")
        
        cap, config = reinit_camera(idx)
        
        if cap is not None and config is not None:
            # Read multiple test frames for better duplicate detection
            test_frames = []
            for _ in range(5):
                ret, test_frame = cap.read()
                if ret and test_frame is not None and test_frame.size > 0:
                    if not is_black_frame(test_frame):
                        test_frames.append(test_frame.copy())
                time.sleep(0.1)
            
            if len(test_frames) < 2:
                print(f"  ⚫ Camera {idx} not producing enough valid frames - SKIPPING")
                cap.release()
                continue
            
            # Use the average of frames for comparison
            avg_frame = np.mean(test_frames, axis=0).astype(np.uint8)
            
            is_duplicate = False
            
            # Compare with existing cameras to detect duplicates
            for existing_idx, ref_data in reference_data.items():
                ref_avg = ref_data['avg_frame']
                
                if ref_avg is not None and avg_frame.shape == ref_avg.shape:
                    # Calculate mean absolute difference
                    diff = cv2.absdiff(avg_frame, ref_avg)
                    mean_diff = np.mean(diff)
                    
                    # If frames are very similar (<15 difference), it's likely a duplicate
                    # Using higher threshold (15) to catch more duplicates
                    # if mean_diff < 15:
                    #     print(f"  ⚠️ Camera {idx} is DUPLICATE of camera {existing_idx} (diff={mean_diff:.1f}) - SKIPPING")
                    #     is_duplicate = True
                    #     cap.release()
                    #     break
                    # else:
                    #     print(f"  ✓ Camera {idx} vs {existing_idx}: diff={mean_diff:.1f} (unique)")
            
            if is_duplicate:
                continue
            
            # Store reference data for future duplicate checks
            reference_data[idx] = {
                'avg_frame': avg_frame,
                'frames': test_frames
            }
            
            # Store camera
            with opencv_global_lock:
                opencv_caps[idx] = cap
                opencv_status[idx] = {"state": "initializing", "reconnects": 0}
            
            # Create stop event for this camera
            stop_event = threading.Event()
            camera_stop_events[idx] = stop_event
            
            # Start reader thread
            thread = threading.Thread(
                target=webcam_reader,
                args=(idx, cap, stop_event),
                daemon=True,
                name=f"CameraReader-{idx}"
            )
            thread.start()
            
            detected_cameras += 1
            print(f"  ✅ Camera {idx} initialized successfully at {config}")
            
            # CRITICAL: Longer delay between camera initializations 
            # to prevent USB bus saturation on USB hubs
            time.sleep(1.0)
        else:
            print(f"  ❌ Camera at index {idx} could not be initialized")
    
    print(f"\n✅ Successfully initialized {detected_cameras} UNIQUE webcam(s)")
    
    # Give all cameras time to produce their first frames
    time.sleep(0.5)



# ============================================================
# GPS
# ============================================================
# ============================================================
# GPS
# ============================================================
def get_gps_serial_port():
    """Attempt to find the correct serial port for GPS/Arduino"""
    system = platform.system()
    
    if system == "Linux":
        # Common Linux serial ports for Arduino/USB-Serial
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
        import glob
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                print(f"📡 Found potential GPS ports: {ports}")
                return ports[0] # Return the first one found
    elif system == "Windows":
        return "COM6" # Fallback to original default for Windows
        
    return None

gps_port = get_gps_serial_port()
print(f"📡 Attempting to connect to GPS on: {gps_port}")

try:
    if gps_port:
        arduino = serial.Serial(gps_port, 9600, timeout=1)
        print(f"✅ Connected to GPS on {gps_port}")
    else:
        print("⚠️ No GPS serial port found")
        arduino = None
except Exception as e:
    print(f"❌ Failed to connect to GPS on {gps_port}: {e}")
    arduino = None

def gps_thread():
    print("📡 GPS thread started")
    while True:
        try:
            if arduino and arduino.in_waiting:
                line = arduino.readline().decode(errors="ignore").strip()
                if line:
                    # print(f"Raw GPS data: {line}") # Uncomment for verbose debug
                    pass
                
                if "Latitude" in line and "Longitude" in line:
                    p = line.split(",")
                    lat = p[0].split(":")[-1].strip()
                    lon = p[1].split(":")[-1].strip()
                    
                    if lat and lon:
                        latest_data["latitude"] = lat
                        latest_data["longitude"] = lon
                        # print(f"📍 GPS Update: {lat}, {lon}")
        except Exception as e:
            print(f"⚠️ GPS Loop Error: {e}")
            time.sleep(1)
        time.sleep(0.1)

# ============================================================
# MJPEG generator - Improved
# ============================================================
def generate_frames(camera_id):
    """
    Generate MJPEG frames with timeout handling.
    
    FIXED: Uses per-camera lock to reduce contention.
    """
    last_frame_time = time.time()
    no_frame_warned = False
    frame_lock = get_frame_lock(camera_id)
    last_frame = None  # Cache last good frame
    
    while True:
        frame = None

        # Use per-camera lock instead of global lock
        with frame_lock:
            frame = opencv_frames.get(camera_id)
            if frame is not None:
                frame = frame.copy()

        if frame is None:
            # Check if we've been waiting too long
            if time.time() - last_frame_time > 3.0:
                if not no_frame_warned:
                    print(f"⚠️ Camera {camera_id} feed timeout - no frames")
                    no_frame_warned = True
                
                # Create a black placeholder frame with text
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"Camera {camera_id}", (80, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
                cv2.putText(placeholder, "No Signal", (90, 140),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
                ok, buf = cv2.imencode(".jpg", placeholder)
                if ok:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" +
                        buf.tobytes() +
                        b"\r\n"
                    )
            time.sleep(0.1)
            continue
        
        # Reset tracking on successful frame
        last_frame_time = time.time()
        no_frame_warned = False
        last_frame = frame

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.tobytes() +
            b"\r\n"
        )
        time.sleep(0.1)  # ~30 FPS limit

# ============================================================
# Routes
# ============================================================
@app.route("/set_resolution/<int:idx>", methods=["POST"])
def set_resolution(idx):
    data = request.json
    width = data.get("width")
    height = data.get("height")
    fps = data.get("fps", 10)
    
    if width and height:
        print(f"📡 Received resolution change request for Camera {idx}: {width}x{height}@{fps}")
        target_resolutions[idx] = (width, height, fps)
        return jsonify({"ok": True})
    return jsonify({"error": "Invalid parameters"}), 400

@app.route("/")
def index():
    cam = request.args.get("camera", default=0, type=int)
    return render_template("index.html", selected_camera=cam)

@app.route("/all_cameras")
def all_cameras():
    return render_template("all_cameras.html")

@app.route("/camera/<int:camera_id>")
def camera_feed(camera_id):
    return Response(
        generate_frames(camera_id),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/available_cameras")
def available_cameras():
    cams = []
    with opencv_global_lock:
        for idx in sorted(opencv_caps.keys()):
            status = opencv_status.get(idx, {}).get("state", "unknown")
            cams.append({"id": idx, "label": f"Webcam {idx}", "status": status})
    return jsonify({"cameras": cams})

@app.route("/capture/<int:camera_id>")
def capture(camera_id):
    frame = None
    frame_lock = get_frame_lock(camera_id)

    with frame_lock:
        frame = opencv_frames.get(camera_id)

    if frame is None:
        return "No frame", 400

    frame = frame.copy()
    cv2.putText(
        frame,
        f"Lat:{latest_data['latitude']} Lon:{latest_data['longitude']}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        2
    )

    _, buf = cv2.imencode(".jpg", frame)
    return send_file(
        io.BytesIO(buf.tobytes()),
        mimetype="image/jpeg",
        as_attachment=True,
        download_name=f"capture_{camera_id}.jpg"
    )

# ============================================================
# Panorama APIs
# ============================================================
@app.route("/panorama/add/<int:camera_id>", methods=["POST"])
def panorama_add(camera_id):
    frame = None
    frame_lock = get_frame_lock(camera_id)

    with frame_lock:
        frame = opencv_frames.get(camera_id)

    if frame is None:
        return jsonify({"error": "No frame"}), 400

    with panorama_lock:
        panorama_frames.append(frame.copy())

    return jsonify({"frames": len(panorama_frames)})

@app.route("/panorama/stitch")
def panorama_stitch():
    with panorama_lock:
        if len(panorama_frames) < 3:
            return "Minimum 3 frames required", 400
        imgs = panorama_frames.copy()
        panorama_frames.clear()

    stitcher = cv2.Stitcher_create()
    status, pano = stitcher.stitch(imgs)

    if status != cv2.Stitcher_OK:
        return "Stitching failed", 500

    cv2.putText(
        pano,
        f"Lat:{latest_data['latitude']} Lon:{latest_data['longitude']}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 0),
        2
    )

    _, buf = cv2.imencode(".jpg", pano)
    return send_file(
        io.BytesIO(buf.tobytes()),
        mimetype="image/jpeg",
        as_attachment=True,
        download_name="panorama.jpg"
    )

# ============================================================
# Data APIs
# ============================================================
@app.route("/data")
def data():
    return jsonify(latest_data)

@app.route("/sensors")
def sensors():
    return jsonify(astrobio_sensors)



@app.route("/gps_map")
def gps_map():
    """Generates and returns a plot of the traveler's path based on stored GPS coordinates."""
    try:
        if not saved_coords:
             # Return a placeholder or empty image if no data
             fig, ax = plt.subplots(figsize=(6, 4))
             ax.text(0.5, 0.5, "No GPS Data Recorded", ha='center', va='center')
             ax.axis('off')
        else:
            # Extract lats/lons
            # Note: saved_coords stores strings like "N/A" or "12.34". Filter valid ones.
            lats = []
            lons = []
            valid_coords = []
            
            for c in saved_coords:
                try:
                    lat = float(c["latitude"])
                    lon = float(c["longitude"])
                    lats.append(lat)
                    lons.append(lon)
                    valid_coords.append((lat, lon))
                except (ValueError, TypeError):
                    continue
            
            if not lats:
                 fig, ax = plt.subplots(figsize=(6, 4))
                 ax.text(0.5, 0.5, "No Valid GPS Data", ha='center', va='center')
                 ax.axis('off')
            else:
                # Set dark background style
                plt.style.use('dark_background')
                
                # SIDEBAR ASPECT RATIO
                fig, ax = plt.subplots(figsize=(6, 5))
                
                # Custom colors from style.css
                # bg: #000, neon-blue: #00bfff, active: #00ff99
                fig.patch.set_facecolor('#0a0a0a') # Card background
                ax.set_facecolor('#000000')        # Plot background
                
                # Plot the path
                ax.plot(lons, lats, color='#00bfff', linewidth=2, label='Path') # Neon blue path
                ax.scatter(lons, lats, c='#00ff99', s=50, zorder=5)  # Neon green points
                
                # Mark start and end
                ax.annotate('Start', (lons[0], lats[0]), textcoords="offset points", xytext=(-10,-10), ha='center', color='white')
                ax.annotate('Current', (lons[-1], lats[-1]), textcoords="offset points", xytext=(10,10), ha='center', color='white')
                
                ax.set_title("Travel Path Data", color='#00bfff')
                ax.set_xlabel("Longitude", color='white')
                ax.set_ylabel("Latitude", color='white')
                ax.grid(True, color='#333333', linestyle='--') # Darker grid
                
                # Color the spines
                ax.spines['bottom'].set_color('#00bfff')
                ax.spines['top'].set_color('#00bfff')
                ax.spines['left'].set_color('#00bfff')
                ax.spines['right'].set_color('#00bfff')
                ax.tick_params(axis='x', colors='white')
                ax.tick_params(axis='y', colors='white')
                
                # Legend with dark background
                legend = ax.legend(facecolor='#0a0a0a', edgecolor='#00bfff')
                for text in legend.get_texts():
                    text.set_color("white")
                
                # Make axis scales equal if possible for map-like appearance, 
                # but careful if variation is tiny
                ax.axis('equal')

        # Save to buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        plt.close(fig)
        buf.seek(0)
        
        return send_file(buf, mimetype='image/png')
        
    except Exception as e:
        print(f"Plotting error: {e}")
        return f"Error generating map: {str(e)}", 500

@app.route("/status")
def status():
    """Debug endpoint to check system status"""
    with opencv_global_lock:
        camera_status = {}
        for idx in list(opencv_caps.keys()) + list(opencv_status.keys()):
            if idx not in camera_status:
                has_frame = idx in opencv_frames
                cap_open = False
                if idx in opencv_caps:
                    try:
                        cap_open = opencv_caps[idx].isOpened()
                    except:
                        pass
                camera_status[idx] = {
                    "has_frame": has_frame,
                    "cap_open": cap_open,
                    "status": opencv_status.get(idx, {})
                }
    
    return jsonify({
        "cameras": camera_status,
        "gps": latest_data,
        "arduino_connected": arduino is not None
    })

@app.route("/reinit_camera/<int:camera_id>", methods=["POST"])
def reinit_camera_route(camera_id):
    """Force reinitialize a specific camera"""
    # Stop existing reader if any
    if camera_id in camera_stop_events:
        camera_stop_events[camera_id].set()
        time.sleep(1)  # Wait for thread to stop
    
    # Release existing capture
    with opencv_global_lock:
        if camera_id in opencv_caps:
            try:
                opencv_caps[camera_id].release()
            except:
                pass
            del opencv_caps[camera_id]
    
    # Reinitialize
    cap, config = reinit_camera(camera_id)
    if cap is not None:
        with opencv_global_lock:
            opencv_caps[camera_id] = cap
            opencv_status[camera_id] = {"state": "running", "reconnects": 0}
        
        stop_event = threading.Event()
        camera_stop_events[camera_id] = stop_event
        
        thread = threading.Thread(
            target=webcam_reader,
            args=(camera_id, cap, stop_event),
            daemon=True,
            name=f"CameraReader-{camera_id}"
        )
        thread.start()
        
        return jsonify({"success": True, "config": config})
    
    return jsonify({"success": False, "error": "Could not reinitialize camera"}), 500

# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    print("🚀 TEAM AUTOMATONS Base Station running")
    print(f"📟 Platform: {platform.system()}")
    print(f"🎥 OpenCV version: {cv2.__version__}")

    threading.Thread(target=gps_thread, daemon=True).start()

    init_webcams()  # Initialize all webcams

    app.run(host="0.0.0.0", port=5005, threaded=True)

    
# .\rs_env\Scripts\Activate.ps1    