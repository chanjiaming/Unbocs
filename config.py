import os

BASE_PATH = "/home/eevee/catkin_ws/src/fz_gemini"
API_KEY = "AIzaSyAojLMeaYC_9CABMf6QHZv5gGV4mWws4ko"
VOSK_MODEL_PATH = os.path.join(BASE_PATH, "vosk-model-small-en-us-0.15")
CAMERA_TOPIC = "/camera/rgb/image_raw"

# /camera/rgb/image_raw
# CAMERA_TOPIC = "/usb_cam/image_raw"
