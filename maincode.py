import cv2
import time
import numpy as np
import serial
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# Load YOLOv8 model
model = YOLO("yolov8l.pt")

# Initialize DeepSORT tracker
tracker = DeepSort(max_age=30)

# Setup serial communication to Arduino
arduino = serial.Serial('/dev/cu.usbmodem12201', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Test message to confirm Arduino connection
arduino.write("System Initialized\n".encode())
time.sleep(0.5)

# Open video stream
cap = cv2.VideoCapture(0)

vehicle_speeds = {}

frame_width = 640  # in pixels
real_world_distance = 10  # meters

VEHICLE_CLASSES = {
    2: "Car",
    3: "Motorcycle",
    5: "Bus",
    7: "Truck",
    1: "Bicycle",
    6: "Train"
}
PERSON_CLASS = 0
HELMET_CLASS_ID = 1

def calculate_speed(vehicle_id, x1, timestamp):
    if vehicle_id in vehicle_speeds:
        prev_x1, prev_time = vehicle_speeds[vehicle_id]
        time_diff = timestamp - prev_time
        if time_diff > 0:
            pixel_distance = abs(x1 - prev_x1)
            meters_per_pixel = real_world_distance / frame_width
            speed = (pixel_distance * meters_per_pixel) / time_diff
            return speed * 3.6  # km/h
    return 0

def is_cyclist_holding_handle(person_box, bicycle_box):
    px1, py1, px2, py2 = person_box
    bx1, by1, bx2, by2 = bicycle_box

    if px1 >= bx1 and px2 <= bx2 and py1 >= by1 and py2 <= by2:
        hand_position_threshold = by1 + (by2 - by1) * 0.3
        if py1 > hand_position_threshold:
            return False
    return True

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    start_time = time.time()

    results = model(frame, conf=0.5)

    detections = []
    persons = []
    bicycles = []
    helmets = []


    for result in results:
        for box in result.boxes.data.tolist():
            x1, y1, x2, y2, conf, class_id = box
            class_id = int(class_id)

            if class_id == PERSON_CLASS:
                persons.append([x1, y1, x2, y2])
            elif class_id == 1:
                bicycles.append([x1, y1, x2, y2])
            elif class_id in VEHICLE_CLASSES:
                vehicle_label = VEHICLE_CLASSES[class_id]
                detections.append([[x1, y1, x2, y2], conf, vehicle_label])
            elif class_id == HELMET_CLASS_ID:
                helmets.append((x1, y1, x2, y2)) 

    tracked_objects = tracker.update_tracks(detections, frame=frame)
    messages_to_send = []

    for track in tracked_objects:
        if not track.is_confirmed():
            continue
        x1, y1, x2, y2 = track.to_tlbr()
        vehicle_id = track.track_id
        label = track.get_det_class()

        speed = calculate_speed(vehicle_id, x1, start_time)
        vehicle_speeds[vehicle_id] = (x1, start_time)

        is_overspeeding = speed > 10
        color = (0, 255, 0) if not is_overspeeding else (0, 0, 255)
        text = f"{label}: {speed:.1f} km/h"
        if is_overspeeding:
            text += "Overspeeding"
            messages_to_send.append(text[:80])  # Trim to max LCD chars
        else:
            messages_to_send.append(text[:80])

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        cv2.putText(frame, text, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


        if label == "Motorcycle":
            associated_persons = []
            helmet_detected = False

            # Use relaxed IOU-like overlap check
            for person in persons:
                px1, py1, px2, py2 = person

                # Check horizontal and vertical overlap with bike
                horizontal_overlap = min(x2, px2) - max(x1, px1)
                vertical_overlap = min(y2, py2) - max(y1, py1)

                if horizontal_overlap > 0 and vertical_overlap > 0:
                    associated_persons.append(person)

            # Count and warn
            person_count = len(associated_persons)

            if person_count > 2:
                warning_text = "Only two people allowed!"
                cv2.putText(frame, warning_text, (int(x1), int(y1) - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                messages_to_send.append(warning_text[:80])

            # Check helmet for the first associated person (assuming rider)
            if person_count >= 1:
                rx1, ry1, rx2, ry2 = associated_persons[0]
                for helmet in helmets:
                    hx1, hy1, hx2, hy2 = helmet
                    if hx1 > rx1 and hx2 < rx2 and hy2 < ry1:
                        helmet_detected = True
                        break

                if not helmet_detected:
                    warning_text = "Please Wear Helmet!"
                    cv2.putText(frame, warning_text, (int(rx1), int(ry1) - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    messages_to_send.append(warning_text[:80])

            for bicycle in bicycles:
                for person in persons:
                    if not is_cyclist_holding_handle(person, bicycle):
                        bx1, by1, bx2, by2 = bicycle
                        warning_text = "Not Holding Handle!"
                        cv2.putText(frame, warning_text, (int(bx1), int(by1) - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        messages_to_send.append(warning_text[:80])

combined_message = ""
for i in range(0, len(messages_to_send), 2):
    line1 = messages_to_send[i][:20]
    line2 = messages_to_send[i + 1][:20] if i + 1 < len(messages_to_send) else ""
    combined_message = line1 + line2
    try:
        arduino.write((combined_message + "\n").encode())
        time.sleep(0.4)
    except Exception as e:
        print("Serial error:", e)

    cv2.imshow("Vehicle Speed & Helmet Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
