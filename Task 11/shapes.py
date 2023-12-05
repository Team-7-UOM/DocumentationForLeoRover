import cv2
import os
import numpy as np

# Global variables
roi_defined = False
selection_start = (0, 0)
selection_end = (0, 0)

# Function to save images to a folder
def save_image(image, folder, shape_name, count):
    file_name = f"{shape_name}_{count}.png"
    file_path = os.path.join(folder, file_name)
    cv2.imwrite(file_path, image)
    print(f"Saved: {file_path}")

# Function to handle mouse events
def select_roi(event, x, y, flags, param):
    global roi_defined, selection_start, selection_end

    if event == cv2.EVENT_LBUTTONDOWN:
        selection_start = (x, y)
        roi_defined = False

    elif event == cv2.EVENT_LBUTTONUP:
        selection_end = (x, y)
        roi_defined = True

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the mouse callback function
cv2.namedWindow("Camera Feed")
cv2.setMouseCallback("Camera Feed", select_roi)

# Create a folder to save the images
folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\multi-shapes"
os.makedirs(folder, exist_ok=True)

# Capture and save images for six orientations
for orientation in range(8):
    for count in range(100):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Display the frame
        cv2.imshow("Camera Feed", frame)

        # Draw the selection rectangle if ROI is being defined
        if roi_defined:
            roi = frame[selection_start[1]:selection_end[1], selection_start[0]:selection_end[0]]
            zoomed_roi = cv2.resize(roi, (frame.shape[1], frame.shape[0]))

            # Convert the zoomed ROI to grayscale
            gray_roi = cv2.cvtColor(zoomed_roi, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to the grayscale image
            blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

            # Apply Canny edge detection
            edges = cv2.Canny(blurred_roi, 30, 30)

            # Display the zoomed-in ROI with edges
            cv2.imshow("Zoomed ROI with Edges", edges)

            # Save the image
            save_image(edges, folder, f"shape_orientation_{orientation}", count)

        # Pause and ask for a change in orientation after every 100 images
        if (count + 1) % 100 == 0:
            print(f"Change the object's orientation and press any key to continue...")
            cv2.waitKey(0)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the VideoCapture and destroy all windows
cap.release()
cv2.destroyAllWindows()
