import cv2
import numpy as np
import time
import threading
from flask import Flask, Response, jsonify, request
from picamera2 import Picamera2

from camera_utils import (
    RED_LOWER1,
    RED_UPPER1,
    RED_LOWER2,
    RED_UPPER2,
    GREEN_LOWER,
    GREEN_UPPER,
    MAGENTA_LOWER,
    MAGENTA_UPPER,
    WHITE_LOWER,
    WHITE_UPPER,
    BLUE_LOWER,
    BLUE_UPPER,
    ORANGE_LOWER,
    ORANGE_UPPER,
)


# ============================================================
# Match your existing camera_utils.py settings
# ============================================================

VIDEO_SIZE = (400, 225)
SENSOR_VIDEO_SIZE = (2304, 1296)
TARGET_FRAME_DURATION_US = 10000

CAMERA_FORMAT = "BGR888"

HOST = "0.0.0.0"
PORT = 5001

JPEG_QUALITY = 75
DEFAULT_PATCH_RADIUS = 3

# True = preview looks like your current camera_utils.py stream/output.
# False = preview looks naturally correct in the browser.
MATCH_LIBRARY_DISPLAY_OUTPUT = True


app = Flask(__name__)

camera = None
latest_frame = None
latest_jpeg = None
frame_lock = threading.Lock()
running = True


def to_int_list(arr):
    return np.round(arr).astype(int).tolist()


def hsv_from_library_interpretation(rgb_like_value):
    """
    This matches your current library behaviour.

    Your camera config is BGR888, but your pipeline does:
        cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    Therefore the raw frame channels are interpreted as if they were RGB.
    This function reproduces that exact behaviour for one RGB-like value.
    """
    pixel = np.uint8([[rgb_like_value]])
    hsv = cv2.cvtColor(pixel, cv2.COLOR_RGB2HSV)[0][0]
    return hsv.astype(int)


def true_hsv_from_bgr(raw_bgr_value):
    """
    This is the physically correct HSV if the camera frame is BGR888.
    This is shown only for comparison.
    It is NOT what your current filter pipeline uses.
    """
    pixel = np.uint8([[raw_bgr_value]])
    hsv = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv.astype(int)


def in_range_hsv(hsv, lower, upper):
    hsv = np.array(hsv, dtype=float)
    lower = np.array(lower, dtype=float)
    upper = np.array(upper, dtype=float)

    return bool(np.all(hsv >= lower) and np.all(hsv <= upper))


def get_filter_matches(hsv):
    matches = []

    if in_range_hsv(hsv, RED_LOWER1, RED_UPPER1):
        matches.append("RED_1")

    if in_range_hsv(hsv, RED_LOWER2, RED_UPPER2):
        matches.append("RED_2")

    if in_range_hsv(hsv, GREEN_LOWER, GREEN_UPPER):
        matches.append("GREEN")

    if in_range_hsv(hsv, MAGENTA_LOWER, MAGENTA_UPPER):
        matches.append("MAGENTA")

    if in_range_hsv(hsv, WHITE_LOWER, WHITE_UPPER):
        matches.append("WHITE")

    if in_range_hsv(hsv, BLUE_LOWER, BLUE_UPPER):
        matches.append("BLUE")

    if in_range_hsv(hsv, ORANGE_LOWER, ORANGE_UPPER):
        matches.append("ORANGE")

    if not matches:
        matches.append("NONE")

    return matches


def get_patch(frame, x, y, radius):
    h, w = frame.shape[:2]

    x1 = max(0, x - radius)
    x2 = min(w, x + radius + 1)
    y1 = max(0, y - radius)
    y2 = min(h, y + radius + 1)

    return frame[y1:y2, x1:x2], (x1, y1, x2, y2)


def camera_loop():
    global latest_frame, latest_jpeg, running

    prev_time = time.time()

    while running:
        frame = camera.capture_array()

        now = time.time()
        fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
        prev_time = now

        if MATCH_LIBRARY_DISPLAY_OUTPUT:
            # Same as your current camera_utils.py:
            # display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            # Natural display for BGR888 frame.
            display_bgr = frame.copy()

        cv2.putText(
            display_bgr,
            f"FPS: {fps:.1f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, buffer = cv2.imencode(
            ".jpg",
            display_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if ok:
            with frame_lock:
                latest_frame = frame.copy()
                latest_jpeg = buffer.tobytes()

        time.sleep(0.001)


def generate_stream():
    while True:
        with frame_lock:
            frame = latest_jpeg

        if frame is None:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )

        time.sleep(0.001)


@app.route("/")
def index():
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Pi Camera RGB / HSV Picker</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #111;
            color: #eee;
            margin: 20px;
        }

        #container {
            display: flex;
            gap: 24px;
            align-items: flex-start;
        }

        #videoBox {
            position: relative;
            display: inline-block;
        }

        #stream {
            border: 2px solid #555;
            image-rendering: pixelated;
            cursor: crosshair;
        }

        #crosshair {
            position: absolute;
            width: 15px;
            height: 15px;
            border: 1px solid yellow;
            border-radius: 50%;
            pointer-events: none;
            display: none;
            transform: translate(-50%, -50%);
        }

        #panel {
            min-width: 430px;
            background: #222;
            padding: 16px;
            border-radius: 8px;
            border: 1px solid #444;
        }

        .value {
            font-family: monospace;
            font-size: 15px;
            margin: 8px 0;
        }

        .important {
            color: #ffff66;
            font-weight: bold;
        }

        .swatch {
            width: 120px;
            height: 80px;
            border: 2px solid #777;
            margin-top: 12px;
        }

        input {
            width: 80px;
        }

        button {
            margin-top: 8px;
            padding: 6px 10px;
        }

        .hint {
            color: #aaa;
            font-size: 14px;
            margin-top: 12px;
            line-height: 1.4;
        }

        pre {
            color: #ddd;
            background: #111;
            padding: 8px;
            max-height: 260px;
            overflow-y: auto;
            border: 1px solid #444;
        }
    </style>
</head>
<body>
    <h2>Pi Camera RGB / HSV Picker</h2>

    <div id="container">
        <div id="videoBox">
            <img id="stream" src="/stream.mjpg">
            <div id="crosshair"></div>
        </div>

        <div id="panel">
            <h3>Current Point</h3>

            <div class="value">XY: <span id="xy">-</span></div>

            <hr>

            <div class="value important">
                Library Avg RGB-like value:
                <span id="libAvgRgb">-</span>
            </div>

            <div class="value important">
                Library Avg HSV:
                <span id="libAvgHsv">-</span>
            </div>

            <div class="value important">
                Filter Match:
                <span id="matches">-</span>
            </div>

            <hr>

            <div class="value">
                Library Pixel RGB-like:
                <span id="libPixelRgb">-</span>
            </div>

            <div class="value">
                Library Pixel HSV:
                <span id="libPixelHsv">-</span>
            </div>

            <hr>

            <div class="value">
                True RGB:
                <span id="trueRgb">-</span>
            </div>

            <div class="value">
                True HSV:
                <span id="trueHsv">-</span>
            </div>

            <hr>

            <div>
                Patch radius:
                <input id="radius" type="number" value="3" min="0" max="30">
            </div>

            <div class="value">Patch size: <span id="patchSize">-</span></div>

            <div id="swatch" class="swatch"></div>

            <button onclick="printCurrent()">Print current value</button>

            <div class="hint">
                Use <b>Library Avg RGB-like value</b> and <b>Library Avg HSV</b>
                if you are tuning your existing camera_utils.py filters.<br><br>
                The <b>True RGB</b> section is only for checking the real visual colour.
            </div>

            <pre id="log"></pre>
        </div>
    </div>

<script>
const img = document.getElementById("stream");
const crosshair = document.getElementById("crosshair");

let latestData = null;
let lastRequestTime = 0;

function getImageCoordinates(event) {
    const rect = img.getBoundingClientRect();

    const displayX = event.clientX - rect.left;
    const displayY = event.clientY - rect.top;

    const scaleX = img.naturalWidth / rect.width;
    const scaleY = img.naturalHeight / rect.height;

    const x = Math.round(displayX * scaleX);
    const y = Math.round(displayY * scaleY);

    return {x, y, displayX, displayY};
}

async function sampleAt(x, y, shouldPrint=false) {
    const radius = document.getElementById("radius").value;

    const response = await fetch(`/sample?x=${x}&y=${y}&r=${radius}&print=${shouldPrint ? 1 : 0}`);
    const data = await response.json();

    if (data.error) {
        console.log(data.error);
        return;
    }

    latestData = data;

    document.getElementById("xy").textContent = `(${data.x}, ${data.y})`;

    document.getElementById("libAvgRgb").textContent = `[${data.library_avg_rgb_like.join(", ")}]`;
    document.getElementById("libAvgHsv").textContent = `[${data.library_avg_hsv.join(", ")}]`;
    document.getElementById("matches").textContent = data.filter_matches.join(", ");

    document.getElementById("libPixelRgb").textContent = `[${data.library_pixel_rgb_like.join(", ")}]`;
    document.getElementById("libPixelHsv").textContent = `[${data.library_pixel_hsv.join(", ")}]`;

    document.getElementById("trueRgb").textContent = `[${data.true_avg_rgb.join(", ")}]`;
    document.getElementById("trueHsv").textContent = `[${data.true_avg_hsv.join(", ")}]`;

    document.getElementById("patchSize").textContent = `${data.patch_width}x${data.patch_height}`;

    const rgb = data.true_avg_rgb;
    document.getElementById("swatch").style.backgroundColor = `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
}

img.addEventListener("mousemove", (event) => {
    const now = Date.now();

    if (now - lastRequestTime < 80) {
        return;
    }

    lastRequestTime = now;

    const pos = getImageCoordinates(event);

    crosshair.style.display = "block";
    crosshair.style.left = `${pos.displayX}px`;
    crosshair.style.top = `${pos.displayY}px`;

    sampleAt(pos.x, pos.y, false);
});

img.addEventListener("mouseleave", () => {
    crosshair.style.display = "none";
});

img.addEventListener("click", (event) => {
    const pos = getImageCoordinates(event);
    sampleAt(pos.x, pos.y, true);
});

function printCurrent() {
    if (!latestData) {
        return;
    }

    const line =
        `XY=(${latestData.x}, ${latestData.y}) | ` +
        `Library Avg RGB-like=[${latestData.library_avg_rgb_like.join(", ")}] | ` +
        `Library Avg HSV=[${latestData.library_avg_hsv.join(", ")}] | ` +
        `Match=${latestData.filter_matches.join(", ")} | ` +
        `True RGB=[${latestData.true_avg_rgb.join(", ")}]`;

    document.getElementById("log").textContent = line + "\\n" + document.getElementById("log").textContent;
}
</script>
</body>
</html>
"""


@app.route("/stream.mjpg")
def stream():
    return Response(
        generate_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/sample")
def sample():
    with frame_lock:
        if latest_frame is None:
            return jsonify({"error": "No frame available yet"})

        frame = latest_frame.copy()

    h, w = frame.shape[:2]

    try:
        x = int(request.args.get("x", 0))
        y = int(request.args.get("y", 0))
        radius = int(request.args.get("r", DEFAULT_PATCH_RADIUS))
    except ValueError:
        return jsonify({"error": "Invalid x/y/r value"})

    radius = max(0, min(radius, 30))

    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))

    raw_pixel = frame[y, x].astype(int)

    # Because the camera is configured as BGR888:
    raw_bgr = raw_pixel.tolist()

    # This is what your current library effectively treats as RGB
    # when it calls cv2.COLOR_RGB2HSV on the raw frame.
    library_pixel_rgb_like = raw_bgr

    # This is the physically correct RGB value.
    true_pixel_rgb = [raw_bgr[2], raw_bgr[1], raw_bgr[0]]

    library_pixel_hsv = to_int_list(
        hsv_from_library_interpretation(library_pixel_rgb_like)
    )

    true_pixel_hsv = to_int_list(
        true_hsv_from_bgr(raw_bgr)
    )

    patch, (x1, y1, x2, y2) = get_patch(frame, x, y, radius)

    avg_raw_bgr = np.mean(patch.reshape(-1, 3), axis=0)
    avg_raw_bgr = np.round(avg_raw_bgr).astype(int).tolist()

    library_avg_rgb_like = avg_raw_bgr
    true_avg_rgb = [avg_raw_bgr[2], avg_raw_bgr[1], avg_raw_bgr[0]]

    library_avg_hsv = to_int_list(
        hsv_from_library_interpretation(library_avg_rgb_like)
    )

    true_avg_hsv = to_int_list(
        true_hsv_from_bgr(avg_raw_bgr)
    )

    matches = get_filter_matches(library_avg_hsv)

    result = {
        "x": x,
        "y": y,
        "image_width": w,
        "image_height": h,

        "raw_bgr": raw_bgr,

        "library_pixel_rgb_like": library_pixel_rgb_like,
        "library_pixel_hsv": library_pixel_hsv,

        "library_avg_rgb_like": library_avg_rgb_like,
        "library_avg_hsv": library_avg_hsv,

        "true_pixel_rgb": true_pixel_rgb,
        "true_pixel_hsv": true_pixel_hsv,

        "true_avg_rgb": true_avg_rgb,
        "true_avg_hsv": true_avg_hsv,

        "filter_matches": matches,

        "patch_radius": radius,
        "patch_width": x2 - x1,
        "patch_height": y2 - y1,
    }

    if request.args.get("print", "0") == "1":
        print(
            f"XY=({x}, {y}) | "
            f"Library Avg RGB-like={library_avg_rgb_like} | "
            f"Library Avg HSV={library_avg_hsv} | "
            f"Match={matches} | "
            f"True RGB={true_avg_rgb} | "
            f"Patch={x2 - x1}x{y2 - y1}"
        )

    return jsonify(result)


def main():
    global camera, running

    print("======= Initializing Web RGB/HSV Picker =======")

    camera = Picamera2()

    config = camera.create_video_configuration(
        main={"size": VIDEO_SIZE, "format": CAMERA_FORMAT},
        sensor={"output_size": SENSOR_VIDEO_SIZE},
        controls={
            "FrameDurationLimits": (
                TARGET_FRAME_DURATION_US,
                TARGET_FRAME_DURATION_US,
            )
        },
        buffer_count=4,
        queue=False,
    )

    camera.configure(config)
    camera.start()

    print(camera.camera_configuration())
    print("Camera: Activated")
    print()
    print(f"Camera format: {CAMERA_FORMAT}")
    print(f"Match library display output: {MATCH_LIBRARY_DISPLAY_OUTPUT}")
    print()
    print(f"Open this on the Pi:       http://localhost:{PORT}")
    print(f"Open this from your Mac:   http://<PI_IP_ADDRESS>:{PORT}")
    print()
    print("Find Pi IP address with:")
    print("hostname -I")
    print("==============================================")

    thread = threading.Thread(target=camera_loop, daemon=True)
    thread.start()

    try:
        app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)
    finally:
        running = False
        time.sleep(0.2)
        camera.stop()
        print("Camera stopped.")


if __name__ == "__main__":
    main()
