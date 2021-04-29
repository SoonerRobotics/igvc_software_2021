import pickle
import numpy as np
import tensorflow as tf
from tensorflow import keras
import cv2

from image_processing import *
from unet import *

# Load the images
ins, ins_filenames = read_images_from_directory("./example/video_frames", "fv_[\d]*.png", processing='in')

# Reload the model
model = tf.keras.models.load_model('./results/SCRUNet_model')

# Make the predictions
predictions = model.predict(ins)

for i, prediction in enumerate(predictions):
    # Post process predictions to make images we can use
    prediction = post_process(prediction, threshold=0.5)

    cv2.imwrite(f"./example/video_prediction/f{i:04}.png", prediction)