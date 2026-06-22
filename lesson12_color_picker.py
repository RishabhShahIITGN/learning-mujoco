import cv2
import numpy as np
import imageio.v2 as imageio

# 1. Load the debug image
# (Make sure this is the one where the ball is in the center!)
image = imageio.imread("robot_view_debug.png")

# 2. Get image dimensions
height, width, channels = image.shape
cx, cy = width // 2, height // 2

# 3. Get the pixel color at the center
pixel_rgb = image[cy, cx]

# 4. Convert just that pixel to HSV
pixel_hsv = cv2.cvtColor(np.uint8([[pixel_rgb]]), cv2.COLOR_RGB2HSV)[0][0]

print("--------------------------------------------------")
print(f"CENTER PIXEL ANALYSIS (Coordinates: {cx}, {cy})")
print("--------------------------------------------------")
print(f"RGB Value: {pixel_rgb}")
print(f"HSV Value: {pixel_hsv}")
print("--------------------------------------------------")
print("Use the HSV values above to fix your lower_green/upper_green range.")