import cv2
import mediapipe as mp

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


cap = cv2.VideoCapture(0)

# STEP 2: Create an GestureRecognizer object.
base_options = python.BaseOptions(model_asset_path='gesture_data/gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)


def thumb_recognizer(frame) -> bool:
    image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
    result = recognizer.recognize(image)

    if len(result.gestures) > 0:
        top_result = result.gestures[0][0]
        # print(f"Gesture {top_result.category_name} has score {top_result.score}")
        return top_result.score > 0.5 and top_result.category_name == "Thumb_Up"
    return False
