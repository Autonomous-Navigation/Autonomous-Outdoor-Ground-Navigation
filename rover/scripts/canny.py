import cv2

# Initialize the camera capture
cap = cv2.VideoCapture(2)  # Use 0 for the default camera, adjust as needed

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale (Canny requires grayscale input)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray, 100, 200)  # Adjust threshold values as needed

    # Display the original frame and the edges
    cv2.imshow('Original', frame)
    cv2.imshow('Edges', edges)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

