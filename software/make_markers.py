"""Generate the ArUco stickers (4x4): ID 0 for the FINGER (top of the finger, 20 x 20 mm) and ID 1 for the BASE/fixed jaw. Print at 300 dpi, no scaling."""
import cv2, numpy as np, sys
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); px = 236            # 20 mm at 300 dpi
for i in (0, 1):
    m = cv2.aruco.generateImageMarker(d, i, px); m = cv2.copyMakeBorder(m, 35, 35, 35, 35, cv2.BORDER_CONSTANT, value=255)   # 3 mm white border
    cv2.imwrite(f"aruco_id{i}_20mm_300dpi.png", m)
print("ok")
