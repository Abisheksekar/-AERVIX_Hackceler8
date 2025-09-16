import cv2
import time
import mediapipe as mp
import math
import os
import asyncio
from threading import Thread
from telegram import Bot, InputFile
import RPi.GPIO as GPIO
import pygame

# Configure GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pin Configurations
ALCOHOL_TEST_PIN = 19  # Pin to initiate alcohol test
ALCOHOL_DETECTION_PIN = 26  # Pin for alcohol detection
DROWSY_INDICATOR_PIN = 12  # Pin for drowsiness indication

# Setup GPIO Pins
GPIO.setup(ALCOHOL_TEST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ALCOHOL_DETECTION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(DROWSY_INDICATOR_PIN, GPIO.OUT, initial=GPIO.LOW)

# Paths and Configuration
VIDEO_SAVE_PATH = '/home/pi/driver_monitoring'
AUDIO_ALERT_PATH = "/home/pi/DROWSY AUDIO/alert.wav"

# Telegram Bot Configuration
BOT_TOKEN = "7823488815:AAHziUt1fA6FeG9DCUTiPZNu7j6Bde6K4VQ"
CHAT_ID = '665062736'

# Ensure video save directory exists
os.makedirs(VIDEO_SAVE_PATH, exist_ok=True)

# Initialize pygame for audio
pygame.mixer.init()

# Initialize Mediapipe Face Mesh
mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5)

# Eye and Face Landmark Configurations
LEFT_EYE_LANDMARKS = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_LANDMARKS = [33, 160, 158, 133, 153, 144]

# Thresholds and Constants
EYE_CLOSURE_THRESHOLD = 0.3
EYE_CLOSURE_FRAMES = 20
MOUTH_OPENING_THRESHOLD = 0.06

"""async def send_video_to_telegram(video_path):
    Send video to Telegram
    bot = Bot(token=BOT_TOKEN)
    try:
        with open(video_path, 'rb') as video_file:
            await bot.send_video(chat_id=CHAT_ID, video=InputFile(video_file))
        print(f"Video sent to Telegram: {video_path}")
    except Exception as e:
        print(f"Failed to send video: {e}")
"""
def play_alert_audio():
    """Play alert audio"""
    pygame.mixer.music.load(AUDIO_ALERT_PATH)
    pygame.mixer.music.play()

def euclidean_distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

def detect_yawning(face_landmarks):
    """Detect yawning based on mouth opening"""
    upper_lip = face_landmarks.landmark[13]
    lower_lip = face_landmarks.landmark[14]
    mouth_opening = euclidean_distance(upper_lip, lower_lip)
    return mouth_opening > MOUTH_OPENING_THRESHOLD

def eye_aspect_ratio(eye_landmarks):
    """Calculate Eye Aspect Ratio"""
    vertical_1 = euclidean_distance(eye_landmarks[1], eye_landmarks[5])
    vertical_2 = euclidean_distance(eye_landmarks[2], eye_landmarks[4])
    horizontal = euclidean_distance(eye_landmarks[0], eye_landmarks[3])

    return (vertical_1 + vertical_2) / (2.0 * horizontal) if horizontal != 0 else 0.0

def perform_alcohol_test():
    """Perform alcohol detection test"""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    timestamp = int(time.time())
    video_filename_name = f"alcohol_test_{timestamp}.avi"
    video_filename = os.path.join(VIDEO_SAVE_PATH, video_filename_name)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20, (640, 480))

    start_time = time.time()
    is_drunken = False

    while time.time() - start_time < 10:
        ret, frame = cap.read()
        if not ret:
            break

        # Display "Blow the air" message
        cv2.putText(frame, "BLOW THE AIR", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        cv2.imshow('Alcohol Test', frame)
        out.write(frame)

        # Check alcohol detection pin
        if GPIO.input(ALCOHOL_DETECTION_PIN) == GPIO.HIGH:
            is_drunken = True
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    if is_drunken:
        # Send video through Telegram
        Thread(target=asyncio.run, args=(send_video_to_telegram(video_filename),)).start()
        print("Vehicle locked! Driver appears to be intoxicated.")
        return False
    return True

def main_drowsy_detection():
    """Main drowsy detection system"""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    drowsiness_detected = False
    eye_closed_frames = 0
    recording = False
    out = None
    video_filename = ""

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            print('Webcam Read Error')
            break

        results = face_mesh.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Draw face landmarks
                mp_drawing.draw_landmarks(
                    image=img,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=drawing_spec,
                    connection_drawing_spec=drawing_spec
                )

                # Eye and Yawning Detection Logic
                left_eye = [face_landmarks.landmark[i] for i in LEFT_EYE_LANDMARKS]
                right_eye = [face_landmarks.landmark[i] for i in RIGHT_EYE_LANDMARKS]

                left_EAR = eye_aspect_ratio(left_eye)
                right_EAR = eye_aspect_ratio(right_eye)
                avg_EAR = (left_EAR + right_EAR) / 2.0

                # Drowsiness Detection
                if avg_EAR < EYE_CLOSURE_THRESHOLD:
                    eye_closed_frames += 1
                else:
                    eye_closed_frames = 0

                yawning = detect_yawning(face_landmarks)

                # Display Alerts
                if yawning:
                    cv2.putText(img, "YAWNING DETECTED!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                if eye_closed_frames >= EYE_CLOSURE_FRAMES:
                    cv2.putText(img, "DROWSY EYES DETECTED!", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                # Trigger Alert and Recording
                if eye_closed_frames >= EYE_CLOSURE_FRAMES or yawning:
                    if not drowsiness_detected:
                        drowsiness_detected = True
                        GPIO.output(DROWSY_INDICATOR_PIN, GPIO.HIGH)
                        play_alert_audio()
                        for i in range(10000):
                            for j in range(10000):
                                pass
                        GPIO.output(DROWSY_INDICATOR_PIN, GPIO.LOW)

                        # Start recording
                        timestamp = int(time.time())
                        video_filename = os.path.join(VIDEO_SAVE_PATH, f"drowsy_{timestamp}.avi")
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        out = cv2.VideoWriter(video_filename, fourcc, 20, (640, 480))
                        recording = True
                        print(f"Recording started: {video_filename}")

                else:
                    if drowsiness_detected:
                        drowsiness_detected = False
                        GPIO.output(DROWSY_INDICATOR_PIN, GPIO.LOW)

                        if recording:
                            out.release()
                            print(f"Recording stopped: {video_filename}")
                            recording = False
                            Thread(target=asyncio.run, args=(send_video_to_telegram(video_filename),)).start()

                # Write frame if recording
                if recording:
                    out.write(img)

        # Show video feed
        cv2.imshow('Webcam', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def driver_monitoring_system():
    """Main driver monitoring system workflow"""
    try:
        # Wait for alcohol test initiation
        while True:
            if GPIO.input(ALCOHOL_TEST_PIN) == GPIO.HIGH:
                print("Alcohol test initiated...")
                
                # Perform alcohol test
                if perform_alcohol_test():
                    print("Driver is sober. Proceeding to drowsy detection...")
                    main_drowsy_detection()
                else:
                    print("Vehicle locked due to potential intoxication.")
                    break

            time.sleep(0.1)  # Prevent high CPU usage
    except KeyboardInterrupt:
        print("Monitoring stopped.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    driver_monitoring_system()
