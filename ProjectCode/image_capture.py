import cv2
import os
from datetime import datetime
from picamera2 import Picamera2
import time

# Change this to the name of the person you're photographing
PERSON_NAME = "kevyn"

def create_folder(name):
    dataset_folder = "dataset"
    os.makedirs(dataset_folder, exist_ok=True)
    person_folder = os.path.join(dataset_folder, name)
    os.makedirs(person_folder, exist_ok=True)
    return person_folder

def capture_photos(name):
    folder = create_folder(name)

    picam2 = Picamera2()

    # Use 3-channel RGB for simpler/accurate color conversion to OpenCV (BGR)
    # 1280x720 gives the ISP enough pixels for nicer color than 640x480.
    preview_config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (1280, 720)}
    )
    picam2.configure(preview_config)
    picam2.start()

    # Let AE/AWB settle for better color/brightness
    time.sleep(2.5)

    # OPTIONAL: lock AWB after it settles for consistency across shots.
    # Comment this line out if lighting changes while you shoot.
    picam2.set_controls({"AwbEnable": False})

    photo_count = 0
    print(f"Taking photos for {name}. Press SPACE to capture, 'q' to quit.")

    try:
        while True:
            frame_rgb = picam2.capture_array()   # RGB888 from ISP

            cv2.imshow('Capture (Pi Cam v2, color-corrected)', frame_rgb)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Space
                photo_count += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{name}_{timestamp}.jpg"
                filepath = os.path.join(folder, filename)
                # Save the BGR image so colors are correct on disk
                cv2.imwrite(filepath, frame_rgb)
                print(f"Photo {photo_count} saved: {filepath}")

            elif key == ord('q'):  # Quit
                break

    finally:
        cv2.destroyAllWindows()
        picam2.stop()
        print(f"Photo capture completed. {photo_count} photos saved for {name}.")

if __name__ == "__main__":
    capture_photos(PERSON_NAME)