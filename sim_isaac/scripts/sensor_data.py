import cv2
import numpy as np

camera_matrix = np.array([[381.8360595703125, 0.0, 320],
   [0.0, 381.8360595703125, 240],
   [0.0, 0.0, 1.0]])

cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]
width = 320
height = 240
aperture_width = 1.92 # 3.94
aperture_height = 1.44 # 2.48
fovx, fovy, focal_length, principal_point, aspect_ratio = cv2.calibrationMatrixValues(camera_matrix,
            (width, height), aperture_width, aperture_height)

x_offset = (width * 0.5 - cx)
x_offset_mm = (aperture_width * x_offset) / width
y_offset = (height * 0.5 - cy)
y_offset_mm = (aperture_height * y_offset) / height

print("  FOVX = ", "{:.6f}".format(fovx))
print("  FOVY = ", "{:.6f}".format(fovy))
print("  Focal length    = ", "{:.6f}".format(focal_length))
print("  Principal point = ", "{:.6f}, {:6f}".format(*principal_point))
print("  Aspect ratio    = ", "{:.6f}".format(aspect_ratio))
print("  X_offset_mm   =", "{:.6f}".format(x_offset_mm))
print("  Y_offset_mm   =", "{:.6f}".format(y_offset_mm))

a = aperture_width/2-principal_point[0]
b = aperture_height/2-principal_point[1]
print(a, b)

#   FOVX =  39.949405
#   FOVY =  32.173794
#   Focal length    =  2.291016
#   Principal point =  1.921500, 1.436789
#   Aspect ratio    =  1.000000
#   X_offset_mm   = -0.961500
#   Y_offset_mm   = -0.716789
# -0.9615003662109374 -0.7167885131835936

# print()
# import cv2
# import numpy as np
# camera_matrix = np.array([[462.05496215820312, 0.00000000000000, 317.85333251953125],
#                            [0.00000000000000, 462.62466430664062, 237.50996398925781],
#                            [0.00000000000000, 0.00000000000000, 1.00000000000000]])
# cx = camera_matrix[0, 2]
# cy = camera_matrix[1, 2]
# width = 640
# height = 480
# aperture_width = 7.07
# aperture_height = 5.3
# fovx, fovy, focal_length, principal_point, aspect_ratio = cv2.calibrationMatrixValues(camera_matrix,
#             (width, height), aperture_width, aperture_height)

# x_offset = (width * 0.5 - cx)
# x_offset_mm = (aperture_width * x_offset) / width
# y_offset = (height * 0.5 - cy)
# y_offset_mm = (aperture_height * y_offset) / height

# print("  FOVX = ", "{:.6f}".format(fovx))
# print("  FOVY = ", "{:.6f}".format(fovy))
# print("  Focal length    = ", "{:.6f}".format(focal_length))
# print("  Principal point = ", "{:.6f}, {:6f}".format(*principal_point))
# print("  Aspect ratio    = ", "{:.6f}".format(aspect_ratio))
# print("  X_offset_mm   =", "{:.6f}".format(x_offset_mm))
# print("  Y_offset_mm   =", "{:.6f}".format(y_offset_mm))

# #   FOVX =  69.408936
# #   FOVY =  54.837611
# #   Focal length    =  5.104263
# #   Principal point =  3.511286, 2.622506
# #   Aspect ratio    =  1.001233
# #   X_offset_mm   = 0.023714
# #   Y_offset_mm   = 0.027494