import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import os
import cv2
import numpy as np
from tqdm import tqdm

# Load and augment data function
def load_and_augment_data(folder, label):
    images = []
    labels = []
    filenames = [filename for filename in os.listdir(folder) if filename.endswith(".png")]
    for filename in tqdm(filenames, desc=f'Loading and augmenting data for label {label}'):
        img_path = os.path.join(folder, filename)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (64, 64))  # Reduce image size

        # Data augmentation - rotate images
        for angle in range(0, 360, 5):  # Rotate by 10 degrees
            # Use OpenCV for rotation
            rotated_img = cv2.warpAffine(
                img, cv2.getRotationMatrix2D((img.shape[1] / 2, img.shape[0] / 2), angle, 1),
                (img.shape[1], img.shape[0]), borderMode=cv2.BORDER_CONSTANT, borderValue=255
            )
            rotated_img = rotated_img.reshape((64, 64, 1))  # Reshape after rotation
            images.append(rotated_img)
            labels.append(label)

    return np.array(images), np.array(labels)

# Load data for each shape
cube_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\cube"
cuboid_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\cuboid"
cylinder_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\cylinder"
cylinder_capped_cuboid_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\cylinder-capped cuboid"
diamond_cylinder_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\diamond cylinder"
empty_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\empty"
multi_shapes_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\multi-shapes"
pyramid_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\pyramid"
semi_cylinder_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\semi-cylinder"
unidentified_shape_folder = r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\unidentified shape"

# Load and augment data for each shape
cube_images, cube_labels = load_and_augment_data(cube_folder, label=0)
cuboid_images, cuboid_labels = load_and_augment_data(cuboid_folder, label=1)
cylinder_images, cylinder_labels = load_and_augment_data(cylinder_folder, label=2)
cylinder_capped_cuboid_images, cylinder_capped_cuboid_labels = load_and_augment_data(cylinder_capped_cuboid_folder, label=3)
diamond_cylinder_images, diamond_cylinder_labels = load_and_augment_data(diamond_cylinder_folder, label=4)
empty_images, empty_labels = load_and_augment_data(empty_folder, label=5)
multi_shapes_images, multi_shapes_labels = load_and_augment_data(multi_shapes_folder, label=6)
pyramid_images, pyramid_labels = load_and_augment_data(pyramid_folder, label=7)
semi_cylinder_images, semi_cylinder_labels = load_and_augment_data(semi_cylinder_folder, label=8)
unidentified_shape_images, unidentified_shape_labels = load_and_augment_data(unidentified_shape_folder, label=9)

# Concatenate and split data into training and validation sets
all_images = np.concatenate((cube_images, cuboid_images, cylinder_images,
                             cylinder_capped_cuboid_images, diamond_cylinder_images,
                             empty_images, multi_shapes_images, pyramid_images,
                             semi_cylinder_images, unidentified_shape_images), axis=0)
all_labels = np.concatenate((cube_labels, cuboid_labels, cylinder_labels,
                             cylinder_capped_cuboid_labels, diamond_cylinder_labels,
                             empty_labels, multi_shapes_labels, pyramid_labels,
                             semi_cylinder_labels, unidentified_shape_labels), axis=0)

X_train, X_val, y_train, y_val = train_test_split(all_images, all_labels, test_size=0.2, random_state=42)

# Define the model architecture
model = models.Sequential([
    layers.InputLayer(input_shape=(64, 64, 1)),  # Adjusted input shape
    layers.Conv2D(32, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Flatten(),
    layers.Dense(128, activation='relu'),
    layers.Dense(10, activation='softmax')  # Number of shapes: 10
])

# Compile the model
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

# Train the model with data augmentation
datagen = ImageDataGenerator(rotation_range=360, fill_mode='constant', cval=0, horizontal_flip=True, vertical_flip=True, zoom_range=0.2)

# Create a flow generator for training data
train_generator = datagen.flow(X_train, y_train, batch_size=128)  # Further reduce batch_size

# Determine the number of steps per epoch
steps_per_epoch = len(X_train) // 128  # Adjust batch_size accordingly

# Train the model batch-wise
model.fit(train_generator, steps_per_epoch=steps_per_epoch, epochs=10, validation_data=(X_val, y_val))

# Save the model in HDF5 format
model.save(r"C:\Users\natha.NSLEGION\Desktop\Shapes\shapes_dataset\model_with_rotation_final.h5")

