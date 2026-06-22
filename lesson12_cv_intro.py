import cv2  # OpenCV library
import numpy as np
import imageio.v2 as imageio # Use v2 to avoid warnings

# 1. Load the "Debug" image you just took
# (Make sure you ran the debug script so this file exists!)
image = imageio.imread("robot_view_debug.png")

# 2. Convert to HSV Color Space
# Robots prefer HSV (Hue, Saturation, Value) over RGB because it separates "Color" from "Brightness"
hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

# 3. Define "Green"
lower_green = np.array([35, 40, 10]) 
upper_green = np.array([85, 255, 255])

# 4. Create a Mask
# This says: "Turn everything black, EXCEPT the green stuff"
mask = cv2.inRange(hsv, lower_green, upper_green)

# 5. Find the Center (Moments)
moments = cv2.moments(mask)
if moments["m00"] > 0:
    # Calculate centroid
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    
    # Draw a red dot on the original image to show we found it
    cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)
    print(f"Target Found at pixel coordinates: X={cx}, Y={cy}")
else:
    print("No green target found!")

# 6. Save the result
imageio.imwrite("robot_vision_result.png", image)
print("Saved 'robot_vision_result.png'. Check it out!")