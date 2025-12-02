import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle
import psutil, csv
import os

# Load pre-trained face encodings
print("[INFO] loading encodings...")
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]
log_file = open("perf_log.csv", "w", newline="")
writer = csv.writer(log_file)
writer.writerow(["timestamp", "fps", "cpu%", "mem%", "tempC"])
def log_performance(fps_now):
    cpu = psutil.cpu_percent(interval=None)
    mem = psutil.virtual_memory().percent
    # Read temperature from the Pi
    temp = 0.0
    try:
        temp_output = os.popen("vcgencmd measure_temp").readline()
        temp = float(temp_output.replace("temp=", "").replace("'C\n", ""))
    
    except:
        temp = temp
        #temp = float(temp_output.replace("temp=", "").replace("'C\n", ""))
    writer.writerow([time.time(), fps_now, cpu, mem, temp])
    log_file.flush()
# Camera: 3-channel BGR to match OpenCV
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}  # lower than 1080p for speed
))
picam2.start()
time.sleep(2)  # let AE/AWB settle

cv_scaler = 4  # must be an integer

face_locations = []
face_encodings = []
face_names = []
frame_count = 0
start_time = time.time()
fps = 0

def process_frame(frame_bgr):
    # Resize for speed
    small_bgr = cv2.resize(frame_bgr, (0, 0), fx=1/cv_scaler, fy=1/cv_scaler)
    # face_recognition expects RGB
    small_rgb = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2RGB)

    locations = face_recognition.face_locations(small_rgb)
    encodings  = face_recognition.face_encodings(small_rgb, locations, model='small')  # faster than 'large'

    names = []
    for enc in encodings:
        matches = face_recognition.compare_faces(known_face_encodings, enc)
        name = "Unknown"
        if len(known_face_encodings):
            dists = face_recognition.face_distance(known_face_encodings, enc)
            best = np.argmin(dists)
            if matches[best]:
                name = known_face_names[best]
        names.append(name)
    return locations, names

def draw_results(frame_bgr, locations, names):
    for (top, right, bottom, left), name in zip(locations, names):
        top *= cv_scaler; right *= cv_scaler; bottom *= cv_scaler; left *= cv_scaler
        cv2.rectangle(frame_bgr, (left, top), (right, bottom), (244, 42, 3), 3)
        cv2.rectangle(frame_bgr, (left-3, top-35), (right+3, top), (244, 42, 3), cv2.FILLED)
        cv2.putText(frame_bgr, name, (left + 6, top - 8), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255,255,255), 1)
    return frame_bgr

def calculate_fps():
    global frame_count, start_time
    frame_count += 1
    elapsed = time.time() - start_time
    if elapsed > 1:
        fps_val = frame_count / elapsed
        frame_count = 0
        start_time = time.time()
        return fps_val
    return 0.0

try:
    while True:
        frame_bgr = picam2.capture_array()  # already BGR
        locations, names = process_frame(frame_bgr)
        display = draw_results(frame_bgr, locations, names)

        fps_now = calculate_fps()
        cv2.putText(display, f"FPS: {fps_now:.1f}", (display.shape[1]-150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow('Video', display)
        log_performance(fps_now)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break





finally:
    cv2.destroyAllWindows()
    picam2.stop()