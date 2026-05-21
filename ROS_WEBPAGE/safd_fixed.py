from flask import Flask, Response, render_template, jsonify, request, send_file
import cv2, threading, time, io, numpy as np, serial, platform, subprocess
from datetime import datetime

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

panorama_frames = []
panorama_lock = threading.Lock()

# ============================================================
# OpenCV Webcams
# ============================================================
opencv_frames = {}
opencv_caps = {}
opencv_lock = threading.Lock()

def webcam_reader(idx, cap):
    """Read frames from webcam with better error handling"""
    consecutive_failures = 0
    max_failures = 50
    
    while True:
        try:
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                consecutive_failures = 0
                with opencv_lock:
                    opencv_frames[idx] = frame.copy()
            else:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print(f"⚠️ Camera {idx} stopped producing frames after {max_failures} failures")
                    break
                time.sleep(0.03)
        except Exception as e:
            print(f"⚠️ Camera {idx} read error: {e}")
            consecutive_failures += 1
            time.sleep(0.1)

def list_video_devices():
    """List all video devices on the system"""
    devices = []
    
    if platform.system() == "Linux":
        try:
            # Use v4l2-ctl to list devices
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                  capture_output=True, text=True)
            print("📹 Video devices found by v4l2-ctl:")
            print(result.stdout)
            
            # Also check /dev/video*
            import glob
            video_devs = sorted(glob.glob("/dev/video*"))
            print(f"\n📹 /dev/video* devices: {video_devs}")
            
            # Extract numbers from /dev/video*
            for dev in video_devs:
                try:
                    num = int(dev.replace("/dev/video", ""))
                    devices.append(num)
                except:
                    pass
                    
        except Exception as e:
            print(f"Could not run v4l2-ctl: {e}")
            # Fallback: just check 0-30
            devices = list(range(30))
    else:
        # Windows: check 0-20
        devices = list(range(20))
    
    return devices

def test_camera_thoroughly(idx, verbose=True):
    """Thoroughly test if a camera at index works"""
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"🔍 Testing Camera Index {idx}")
        print(f"{'='*60}")
    
    # Try opening with different backends
    backends = []
    if platform.system() == "Windows":
        backends = [cv2.CAP_DSHOW, cv2.CAP_ANY]
    else:
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
    
    for backend in backends:
        if verbose:
            backend_name = "CAP_DSHOW" if backend == cv2.CAP_DSHOW else \
                          "CAP_V4L2" if backend == cv2.CAP_V4L2 else "CAP_ANY"
            print(f"\n   Trying backend: {backend_name}")
        
        try:
            cap = cv2.VideoCapture(idx, backend)
            
            if not cap.isOpened():
                if verbose:
                    print(f"   ❌ Failed to open")
                cap.release()
                continue
            
            if verbose:
                print(f"   ✅ Opened successfully")
            
            # Try different format combinations
            test_configs = [
                (160, 120, 10, 'MJPG'),   # Ultra low
                # (320, 240, 10, 'MJPG')   # Low
                # (320, 240, 10, 'YUYV'),   # Low YUYV
                # (640, 480, 10, 'MJPG'),   # Medium
            ]
            
            for width, height, fps, fmt in test_configs:
                if verbose:
                    print(f"\n   Testing: {width}x{height} @ {fps}fps ({fmt})")
                
                # Set format
                fourcc = cv2.VideoWriter_fourcc(*fmt)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Wait for camera to apply settings
                time.sleep(0.5)
                
                # Get actual settings
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = int(cap.get(cv2.CAP_PROP_FPS))
                
                if verbose:
                    print(f"      Camera reported: {actual_w}x{actual_h} @ {actual_fps}fps")
                
                # Try reading frames
                frame_success = False
                for attempt in range(15):  # More attempts
                    ret, frame = cap.read()
                    
                    if ret and frame is not None and frame.size > 0:
                        if verbose:
                            print(f"      ✅ SUCCESS! Got frame: {frame.shape}")
                        frame_success = True
                        return cap, (actual_w, actual_h, actual_fps, fmt)
                    
                    time.sleep(0.1)
                
                if verbose:
                    print(f"      ❌ Could not read frames (tried 15 times)")
            
            cap.release()
            
        except Exception as e:
            if verbose:
                print(f"   ❌ Exception: {e}")
            continue
    
    if verbose:
        print(f"\n   ❌ FAILED: Camera at index {idx} could not produce frames")
    return None, None

def init_webcams(max_index=None):
    """Initialize webcams with comprehensive detection"""
    
    print("\n" + "="*60)
    print("🚀 COMPREHENSIVE CAMERA DETECTION")
    print("="*60)
    
    # Get list of potential camera indices
    if max_index is None:
        device_indices = list_video_devices()
    else:
        device_indices = list(range(max_index))
    
    print(f"\n📹 Will test indices: {device_indices[:10]}..." if len(device_indices) > 10 else f"\n📹 Will test indices: {device_indices}")
    
    detected_cameras = 0
    
    for idx in device_indices:
        cap, config = test_camera_thoroughly(idx, verbose=True)
        
        if cap is not None and config is not None:
            opencv_caps[idx] = cap
            
            # Start reader thread
            threading.Thread(
                target=webcam_reader,
                args=(idx, cap),
                daemon=True
            ).start()
            
            detected_cameras += 1
            print(f"\n✅✅✅ Camera {idx} SUCCESSFULLY INITIALIZED ✅✅✅")
            
            # Don't delay too much between cameras
            time.sleep(0.3)
        
        # Small delay between tests
        time.sleep(0.2)
    
    print("\n" + "="*60)
    print(f"🎉 DETECTION COMPLETE: {detected_cameras} cameras initialized")
    print(f"📋 Working camera indices: {sorted(opencv_caps.keys())}")
    print("="*60 + "\n")

# ============================================================
# GPS
# ============================================================
try:
    arduino = serial.Serial("COM6", 9600, timeout=1)
except:
    arduino = None

def gps_thread():
    while True:
        try:
            if arduino and arduino.in_waiting:
                line = arduino.readline().decode(errors="ignore")
                if "Latitude" in line and "Longitude" in line:
                    p = line.split(",")
                    latest_data["latitude"] = p[0].split(":")[-1].strip()
                    latest_data["longitude"] = p[1].split(":")[-1].strip()
        except:
            pass
        time.sleep(0.5)

# ============================================================
# MJPEG generator
# ============================================================
def generate_frames(camera_id):
    while True:
        frame = None

        with opencv_lock:
            frame = opencv_frames.get(camera_id)

        if frame is None:
            time.sleep(0.05)
            continue

        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.tobytes() +
            b"\r\n"
        )

# ============================================================
# Routes
# ============================================================
@app.route("/")
def index():
    cam = request.args.get("camera", default=0, type=int)
    # If camera doesn't exist, use first available
    with opencv_lock:
        if cam not in opencv_caps and opencv_caps:
            cam = min(opencv_caps.keys())
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
    with opencv_lock:
        for idx in sorted(opencv_caps.keys()):
            cams.append({"id": idx, "label": f"Webcam {idx}"})
    return jsonify({"cameras": cams})

@app.route("/capture/<int:camera_id>")
def capture(camera_id):
    frame = None

    with opencv_lock:
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

@app.route("/panorama/add/<int:camera_id>", methods=["POST"])
def panorama_add(camera_id):
    frame = None

    with opencv_lock:
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

@app.route("/data")
def data():
    return jsonify(latest_data)

@app.route("/sensors")
def sensors():
    return jsonify(astrobio_sensors)

@app.route("/store_coord", methods=["POST"])
def store_coord():
    d = request.json
    saved_coords.append({
        "latitude": d["latitude"],
        "longitude": d["longitude"],
        "ts": datetime.now().strftime("%H:%M:%S")
    })
    return jsonify({"ok": True})

# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    print("🚀 TEAM AUTOMATONS Base Station - Enhanced Camera Detection")

    threading.Thread(target=gps_thread, daemon=True).start()

    # Initialize with comprehensive detection
    init_webcams()  # Will auto-detect available indices

    app.run(host="0.0.0.0", port=5005, threaded=True)