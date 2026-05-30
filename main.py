import cv2
import os
import time
from datetime import datetime
from dotenv import load_dotenv

load_dotenv(override=True)

from RekognitionController import check_for_phone
from BoxController import upload_to_box
from EmailController import send_alert_email

CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '1'))
CHECK_INTERVAL = float(os.getenv('CHECK_INTERVAL', '2.0'))  # seconds between Rekognition calls
COOLDOWN = float(os.getenv('COOLDOWN', '30.0'))             # seconds between alerts


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera at index {CAMERA_INDEX}. Try setting CAMERA_INDEX in .env")
        return

    print("RoombaRat is watching. Press 'q' to quit.")

    last_check = 0.0
    last_alert = 0.0
    phone_detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Warning: dropped frame")
            continue

        now = time.time()

        if now - last_check >= CHECK_INTERVAL:
            last_check = now
            try:
                phone_detected = check_for_phone(frame)
            except Exception as e:
                print(f"Rekognition error: {e}")

            if phone_detected and (now - last_alert) >= COOLDOWN:
                last_alert = now
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"phone_caught_{timestamp}.jpg"
                _, buf = cv2.imencode('.jpg', frame)
                image_bytes = buf.tobytes()

                print(f"PHONE DETECTED — uploading to Box and alerting...")
                try:
                    box_url = upload_to_box(image_bytes, filename)
                    send_alert_email(filename, box_url)
                    print(f"Alert sent. Box URL: {box_url}")
                except Exception as e:
                    print(f"Alert failed: {e}")

        color = (0, 0, 255) if phone_detected else (0, 255, 0)
        label = "PHONE DETECTED!" if phone_detected else "Monitoring..."
        cv2.putText(frame, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2)
        cv2.imshow('RoombaRat', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
