import cv2
import time
import os
from ultralytics import YOLO
import serial

# Load the YOLO model
model = YOLO("best228.pt")  # Ensure "best228.pt" is the correct trained model for weed detection.

# Open the laptop's default camera
video_path = 0  # 0 for the default laptop camera
cap = cv2.VideoCapture(video_path)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Initialize serial communication with Arduino
arduino_port = "COM13"  # Replace with your Arduino's port
baud_rate = 9600       # Must match the Arduino baud rate

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print(f"Connected to Arduino on {arduino_port}")
except serial.SerialException as e:
    print(f"Error connecting to Arduino: {e}")
    exit()

# Initialize variables
detection_start_time = None
DETECTION_TIME_THRESHOLD = 0.5 # Seconds to wait before sending data to Arduino

# Create a directory to store captured images
image_dir = "weed_images"
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

# Map weed class IDs to their respective indexes
weed_index_mapping = {
    "goosegrass": 1,
    "crabgrass": 2,
    "nutsedge": 3
}

# Loop through the video frames
while True:
    success, frame = cap.read()
    if not success:
        print("Error: Failed to capture frame. Exiting...")
        break

    # Run YOLO inference on the frame for weed detection
    results = model(frame, verbose=False)

    # Process results
    weed_detected = False
    weed_coordinates = []

    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Extract box coordinates and label
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            class_id = int(box.cls.item())  # Convert class tensor to integer
            confidence = box.conf.item()   # Convert confidence tensor to float

            # Normalize coordinates
            norm_x1 = x1 / frame.shape[1]
            norm_y1 = y1 / frame.shape[0]
            norm_x2 = x2 / frame.shape[1]
            norm_y2 = y2 / frame.shape[0]

            # Only consider weeds with confidence >= 0.75
            if confidence >= 0.7:
                weed_detected = True
                weed_coordinates.append((class_id, confidence, norm_x1, norm_y1, norm_x2, norm_y2))

                # Draw bounding box and label on frame
                label = f"{model.names[class_id]} {confidence:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

    # Handle Arduino communication
    if weed_detected:
        if detection_start_time is None:
            detection_start_time = time.time()
        elif time.time() - detection_start_time > DETECTION_TIME_THRESHOLD:
            print("Weeds detected after threshold time. Sending center coordinates and indexes to Arduino.")
            for coord in weed_coordinates:
                # Calculate the center of the area
                norm_x_center = (coord[2] + coord[4]) / 2  # Center X (Normalized)
                norm_y_center = (coord[3] + coord[5]) / 2  # Center Y (Normalized)

                # Determine the weed type and its index
                class_id, confidence, norm_x1, norm_y1, norm_x2, norm_y2 = coord
                weed_type = model.names[class_id]
                weed_index = weed_index_mapping.get(weed_type, 0)  # Default to 0 if not mapped

                # Send normalized center coordinates and weed index as a comma-separated string
                message = f"{norm_x_center:.3f},{norm_y_center:.3f},{weed_index}\n"
                ser.write(message.encode())  # Send to Arduino
                print(f"Sent to Arduino: {message.strip()}")

                # Send just the index to Arduino for pump control
                index_message = f"{weed_index}\n"
                ser.write(index_message.encode())  # New part: Send the index
                print(f"Sent index to Arduino for pump control: {weed_index}")

                # Print detected weed details
                print(f"Detected weed: {weed_type} (Confidence: {confidence:.2f}, Index: {weed_index})")
                print(f"Center (Normalized): ({norm_x_center:.3f}, {norm_y_center:.3f})")

                # Capture and save the image after detection
                image_filename = os.path.join(image_dir, f"weed_detected_{int(time.time())}.jpg")
                cv2.imwrite(image_filename, frame)
                print(f"Captured image saved as {image_filename}")

            break  # Stop video after the first weed detection and image capture

    else:
        detection_start_time = None

    # Display the frame with detections
    cv2.imshow("YOLO Weed Detection", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
ser.close()
