from flask import Flask, Response, render_template, jsonify, request, send_file
import cv2, threading, time, io, numpy as np, platform
from datetime import datetime
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# Intel RealSense SDK
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("⚠️ pyrealsense2 not installed. Depth camera will not be available.")

# ============================================================
# Flask setup
# ============================================================
app = Flask(__name__)
app.secret_key = "team_automatons_secret_key"

# ============================================================
# Global state
# ============================================================
latest_data = {"latitude": "N/A", "longitude": "N/A"}
saved_coords = []

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
# Intel RealSense D435i state
# ============================================================
realsense_frames = {
    "color": None,       # RGB color frame
    "depth": None,       # Colorized depth frame for visualization
    "depth_raw": None,   # Raw depth data (for processing)
}
realsense_pipeline = None
realsense_lock = threading.Lock()
realsense_stop_event = threading.Event()
realsense_status = {"state": "not_initialized", "error": None}

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
    Reinitialize a single camera with optimized configuration attempts.
    Prioritizes high-resolution MJPEG to match USB bandwidth limits,
    then uncompressed YUYV for lower resolutions.
    """
    if configs is None:
        # Format: (width, height, fps, fourcc_str)
        # 1. High Res MJPEG (Bandwidth efficient)
        # 2. Medium Res YUYV (Uncompressed, good quality)
        # 3. Fallbacks
        configs = [
            (1920, 1080, 30, 'MJPG'),
            (1280, 720,  30, 'MJPG'),
            (800,  600,  30, 'YUYV'),
            (640,  480,  30, 'YUYV'),
            (640,  480,  30, 'MJPG'),  # Safe fallback
            (320,  240,  30, 'MJPG')   # Last resort
        ]
    
    # Try multiple backends
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
            
        for width, height, fps, fourcc_str in configs:
            try:
                # 1. Set FourCC first (Critical for bandwidth)
                fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
                
                # 2. Set Resolution & FPS
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                
                # 3. Disable internal buffer (latency)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # 4. Best-effort Auto Exposure / White Balance
                # (Don't check return values, driver dependent)
                try:
                    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # 3 often equals 'Auto' in V4L2
                    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
                except:
                    pass
                
                # Give camera time to apply settings
                time.sleep(0.5)
                
                # 5. STRICT VERIFICATION
                actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
                
                # Note: Some drivers invoke slight resolution rounding, so exact match is best but 
                # strictly rejecting anything that isn't close is key.
                if int(actual_w) != width or int(actual_h) != height:
                    print(f"⚠️ Camera {idx} rejected: Wanted {width}x{height}, got {int(actual_w)}x{int(actual_h)}")
                    continue
                
                # Verify FourCC if possible (Linux V4L2 is usually reliable, Windows DSHOW tricky)
                # We skip strict FourCC check on Windows DSHOW as it can report 0 sometimes even if working
                if platform.system() != "Windows" and actual_fourcc != fourcc:
                     print(f"⚠️ Camera {idx} rejected: FourCC mismatch")
                     continue

                # Flush buffer
                for _ in range(3):
                    cap.read()
                    time.sleep(0.05)
                
                # 6. READ CHECK (Must not be black)
                valid_frames = 0
                for attempt in range(10):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        if not is_black_frame(frame):
                            valid_frames += 1
                            if valid_frames >= 2:
                                backend_name = "DSHOW" if backend == cv2.CAP_DSHOW else "V4L2"
                                print(f"✅ Camera {idx} initialized: {width}x{height}@{fps} ({fourcc_str}) via {backend_name}")
                                return cap, (width, height, fps)
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"⚠️ Camera {idx} config {width}x{height}@{fps} failed: {e}")
                continue
        
        # Backend failed for all configs
        try:
            cap.release()
        except:
            pass
    
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
    
    # Update status
    with opencv_global_lock:
        opencv_status[idx] = {"state": "running", "reconnects": 0}
    
    while not stop_event.is_set():
        try:
            # Check if camera is valid
            if current_cap is None or not current_cap.isOpened():
                raise Exception("Camera not available")
            
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


def init_webcams(max_index=15):
    """
    Initialize webcams with proper USB hub support.
    
    FIXED: Uses staggered initialization to prevent USB bandwidth saturation.
    """
    print("🔍 Scanning for cameras...")
    
    # First, do a quick scan to find which indices have cameras
    potential_cameras = []
    backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_V4L2
    
    for i in range(max_index):
        try:
            cap = cv2.VideoCapture(i, backend)
            if cap.isOpened():
                potential_cameras.append(i)
            cap.release()
        except:
            pass
        time.sleep(0.1)  # Small delay between scans
    
    print(f"📷 Found {len(potential_cameras)} potential camera(s) at indices: {potential_cameras}")
    
    if not potential_cameras:
        print("⚠️ No cameras detected!")
        return
    
    # Initialize cameras one at a time with proper delays
    # This is crucial for USB hubs to prevent bandwidth conflicts
    detected_cameras = 0
    
    for idx in potential_cameras:
        print(f"🔄 Initializing camera {idx}...")
        
        cap, config = reinit_camera(idx)
        
        if cap is not None and config is not None:
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
            
            # CRITICAL: Longer delay between camera initializations 
            # to prevent USB bus saturation on USB hubs
            time.sleep(1.0)
        else:
            print(f"❌ Camera at index {idx} could not be initialized")
    
    print(f"\n✅ Successfully initialized {detected_cameras} webcam(s)")
    
    # Give all cameras time to produce their first frames
    time.sleep(0.5)


# ============================================================
# Intel RealSense D435i Functions
# ============================================================
def init_realsense():
    """Initialize Intel RealSense D435i depth camera"""
    global realsense_pipeline, realsense_status
    
    if not REALSENSE_AVAILABLE:
        realsense_status = {"state": "not_available", "error": "pyrealsense2 not installed"}
        print("⚠️ RealSense SDK not available")
        return False
    
    try:
        # Create pipeline and config
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Configure streams - D435i supports up to 1280x720 @ 30fps
        # Using lower resolution for better performance with USB hub
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start the pipeline
        profile = pipeline.start(config)
        
        # Get device info
        device = profile.get_device()
        device_name = device.get_info(rs.camera_info.name)
        serial_number = device.get_info(rs.camera_info.serial_number)
        
        print(f"✅ RealSense camera initialized: {device_name} (S/N: {serial_number})")
        
        # Create align object to align depth to color frame
        align = rs.align(rs.stream.color)
        
        # Store pipeline globally
        realsense_pipeline = pipeline
        realsense_status = {"state": "running", "error": None, "device": device_name}
        
        # Start reader thread
        realsense_stop_event.clear()
        thread = threading.Thread(
            target=realsense_reader,
            args=(pipeline, align),
            daemon=True,
            name="RealSenseReader"
        )
        thread.start()
        
        return True
        
    except Exception as e:
        realsense_status = {"state": "error", "error": str(e)}
        print(f"❌ RealSense initialization failed: {e}")
        return False


def realsense_reader(pipeline, align):
    """Read frames from Intel RealSense D435i camera"""
    global realsense_frames, realsense_status
    
    # Create colorizer for depth visualization
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 0)  # Jet color scheme
    
    consecutive_failures = 0
    max_failures = 30
    
    print("🎥 RealSense reader thread started")
    
    while not realsense_stop_event.is_set():
        try:
            # Wait for frames with timeout
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            
            if frames is None:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print("⚠️ RealSense too many failures, stopping...")
                    break
                continue
            
            # Align depth to color frame
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                consecutive_failures += 1
                continue
            
            # Reset failure counter on success
            consecutive_failures = 0
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Colorize depth for visualization
            colorized_depth = colorizer.colorize(depth_frame)
            depth_image = np.asanyarray(colorized_depth.get_data())
            
            # Store frames thread-safely
            with realsense_lock:
                realsense_frames["color"] = color_image.copy()
                realsense_frames["depth"] = depth_image.copy()
            
            # Small throttle
            time.sleep(0.01)
            
        except Exception as e:
            if realsense_stop_event.is_set():
                break
            print(f"⚠️ RealSense reader error: {e}")
            consecutive_failures += 1
            if consecutive_failures >= max_failures:
                break
            time.sleep(0.1)
    
    # Cleanup
    print("🔴 RealSense reader thread stopping")
    try:
        pipeline.stop()
    except:
        pass
    realsense_status["state"] = "stopped"


def generate_realsense_frames(stream_type="color"):
    """Generate MJPEG frames from RealSense camera"""
    last_frame_time = time.time()
    no_frame_warned = False
    
    while True:
        frame = None
        
        with realsense_lock:
            frame = realsense_frames.get(stream_type)
            if frame is not None:
                frame = frame.copy()
        
        if frame is None:
            if time.time() - last_frame_time > 3.0:
                if not no_frame_warned:
                    print(f"⚠️ RealSense {stream_type} feed timeout")
                    no_frame_warned = True
                
                # Create placeholder
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"RealSense {stream_type.title()}", (40, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)
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
        
        last_frame_time = time.time()
        no_frame_warned = False
        
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            continue
        
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.tobytes() +
            b"\r\n"
        )
        time.sleep(0.033)  # ~30 FPS limit


# ============================================================
# GPS (ROS Node)
# ============================================================
class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',  # replace with your GPS topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: NavSatFix):
        # Update global latest_data dict
        latest_data["latitude"] = f"{msg.latitude:.6f}"
        latest_data["longitude"] = f"{msg.longitude:.6f}"

def start_gps_node():
    rclpy.init(args=None)
    node = GPSSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

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

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.tobytes() +
            b"\r\n"
        )
        time.sleep(0.033)  # ~30 FPS limit

# ============================================================
# Routes
# ============================================================
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
    
    # Add regular USB webcams
    with opencv_global_lock:
        for idx in sorted(opencv_caps.keys()):
            status = opencv_status.get(idx, {}).get("state", "unknown")
            cams.append({"id": idx, "label": f"Webcam {idx}", "status": status, "type": "webcam"})
    
    # Add RealSense cameras (if available)
    if REALSENSE_AVAILABLE and realsense_status.get("state") == "running":
        cams.append({
            "id": "realsense_color", 
            "label": "RealSense Color", 
            "status": "running",
            "type": "realsense"
        })
        cams.append({
            "id": "realsense_depth", 
            "label": "RealSense Depth", 
            "status": "running",
            "type": "realsense"
        })
    
    return jsonify({"cameras": cams})

# ============================================================
# RealSense Routes
# ============================================================
@app.route("/realsense/color")
def realsense_color_feed():
    """Stream RealSense color (RGB) frames"""
    return Response(
        generate_realsense_frames("color"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/realsense/depth")
def realsense_depth_feed():
    """Stream RealSense depth frames (colorized)"""
    return Response(
        generate_realsense_frames("depth"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/realsense/status")
def realsense_status_route():
    """Get RealSense camera status"""
    return jsonify({
        "available": REALSENSE_AVAILABLE,
        "status": realsense_status,
        "has_color_frame": realsense_frames.get("color") is not None,
        "has_depth_frame": realsense_frames.get("depth") is not None
    })

@app.route("/realsense/reinit", methods=["POST"])
def realsense_reinit():
    """Reinitialize RealSense camera"""
    global realsense_pipeline
    
    # Stop existing
    realsense_stop_event.set()
    time.sleep(1)
    
    if realsense_pipeline:
        try:
            realsense_pipeline.stop()
        except:
            pass
        realsense_pipeline = None
    
    # Reinitialize
    success = init_realsense()
    return jsonify({"success": success, "status": realsense_status})

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
        (0, 255, 255),
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
        (0, 255, 255),
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

@app.route("/store_coord", methods=["POST"])
def store_coord():
    d = request.json
    if not d:
        return jsonify({"error": "No JSON data"}), 400
    saved_coords.append({
        "latitude": d.get("latitude", "N/A"),
        "longitude": d.get("longitude", "N/A"),
        "ts": datetime.now().strftime("%H:%M:%S")
    })
    return jsonify({"ok": True})

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
        "arduino_connected": False
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
    print(f"🔷 RealSense SDK: {'Available' if REALSENSE_AVAILABLE else 'Not installed'}")

    threading.Thread(target=start_gps_node, daemon=True).start()

    # Initialize all USB webcams first
    init_webcams()
    
    # Initialize Intel RealSense D435i depth camera
    if REALSENSE_AVAILABLE:
        print("\n🔷 Initializing Intel RealSense D435i...")
        init_realsense()

    app.run(host="0.0.0.0", port=5000, threaded=True)