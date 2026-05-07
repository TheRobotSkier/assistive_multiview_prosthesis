#!/usr/bin/env python3
import os

import cv2

# A4 at 300 DPI
A4_W_PX = 2480
A4_H_PX = 3508

# Board parameters
squares_x = 5
squares_y = 7
square_length_m = 0.035  # 35 mm
marker_length_m = 0.026  # 26 mm

# Use a small dictionary first
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Newer OpenCV API
try:
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_length_m, marker_length_m, aruco_dict
    )
except Exception:
    # Older API fallback
    board = cv2.aruco.CharucoBoard_create(
        squares_x, squares_y, square_length_m, marker_length_m, aruco_dict
    )

# Margins on A4
margin_px = 150
board_w_px = A4_W_PX - 2 * margin_px
board_h_px = A4_H_PX - 2 * margin_px

# Generate board image
try:
    board_img = board.generateImage(
        (board_w_px, board_h_px), marginSize=0, borderBits=1
    )
except Exception:
    board_img = board.draw((board_w_px, board_h_px), marginSize=0, borderBits=1)

# Put board on white A4 canvas
canvas = 255 * (cv2.UMat(A4_H_PX, A4_W_PX, cv2.CV_8UC1).get())
y0 = (A4_H_PX - board_img.shape[0]) // 2
x0 = (A4_W_PX - board_img.shape[1]) // 2
canvas[y0 : y0 + board_img.shape[0], x0 : x0 + board_img.shape[1]] = board_img

cwd = os.getcwd()
out_dir = os.path.join(cwd, "charuco_boards_5x7_A4_35x26mm")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "charuco_5x7_A4_35x26mm.png")
cv2.imwrite(out_path, canvas)

print("Saved:", out_path)
print("Board parameters:")
print(f"  squaresX = {squares_x}")
print(f"  squaresY = {squares_y}")
print(f"  squareLength = {square_length_m} m")
print(f"  markerLength = {marker_length_m} m")
print("Print at 100% scale with no page scaling.")
