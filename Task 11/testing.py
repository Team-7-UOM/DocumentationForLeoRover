import tensorflow as tf
import cv2
import numpy as np

# Load the trained model
model = tf.keras.models.load_model(r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\model.h5")

# Function to preprocess an image
def preprocess_image(img):
    img = cv2.resize(img, (224, 224))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = img.reshape((1, 224, 224, 1))
    return img

# Map class indices to shape names
shape_names_mapping = {
    0: 'cube',
    1: 'cuboid',
    2: 'cylinder',
    3: 'cylinder_capped_cuboid',
    4: 'diamond_cylinder',
    5: 'empty',
    6: 'multi_shapes',
    7: 'pyramid',
    8: 'semi_cylinder',
    9: 'unidentified_shape'
}

# Global variables
drawing = False
roi_start = (0, 0)
roi_end = (0, 0)

# Mouse callback function
def draw_roi(event, x, y, flags, param):
    global roi_start, roi_end, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        roi_start = (x, y)
        drawing = True

    elif event == cv2.EVENT_LBUTTONUP:
        roi_end = (x, y)
        drawing = False
        cv2.rectangle(frame, roi_start, roi_end, (0, 255, 0), 2)

# Open a connection to the camera (0 is the default camera)
cap = cv2.VideoCapture(0)

# Create a window and set the mouse callback
cv2.namedWindow('Real-Time Shape Identification')
cv2.setMouseCallback('Real-Time Shape Identification', draw_roi)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the frame to Canny edges only within the ROI
    if not drawing:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

        # Check if the ROI has a non-zero width and height
        if roi_start[0] < roi_end[0] and roi_start[1] < roi_end[1]:
            edges_frame = cv2.Canny(blurred_frame[roi_start[1]:roi_end[1], roi_start[0]:roi_end[0]], 30, 30)

            # Convert edges_frame to a 3-channel image
            edges_frame_bgr = cv2.cvtColor(edges_frame, cv2.COLOR_GRAY2BGR)
            
            # Assign edges_frame_bgr to the specified region in the frame
            frame[roi_start[1]:roi_end[1], roi_start[0]:roi_end[0]] = edges_frame_bgr

    # Preprocess the frame
    input_image = preprocess_image(frame)

    # Make predictions
    predictions = model.predict(input_image)

    # Get the predicted class
    predicted_class = np.argmax(predictions)

    # Get the predicted shape name
    predicted_shape = shape_names_mapping.get(predicted_class, 'unknown')

    # Display the frame with the selected ROI and predicted shape
    cv2.putText(frame, f"Predicted Shape: {predicted_shape}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Real-Time Shape Identification', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
