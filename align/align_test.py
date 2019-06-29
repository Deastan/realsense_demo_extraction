import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
print("Environment Ready")

# Setup:
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("../object_detection.bag")
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(50):
  pipe.wait_for_frames()
  
# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()


# Cleanup:
pipe.stop()
print("Frames Captured")


# # Plot the color map
color = np.asanyarray(color_frame.get_data())
# plt.rcParams["axes.grid"] = False
# plt.rcParams['figure.figsize'] = [12, 6]
# plt.imshow(color)
# plt.show()

# # Plot the depth map
colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
# plt.imshow(colorized_depth)
# plt.show()

# Create alignment primitive with color as its target stream:
align = rs.align(rs.stream.color)
frameset = align.process(frameset)

# Update color and depth frames:
aligned_depth_frame = frameset.get_depth_frame()
colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

# Show the two frames together:
images = np.hstack((color, colorized_depth))
plt.imshow(images)
plt.show()


# From API: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
# The following shows how to retrieve the depth of a pixel in meters:
# dpt_frame = pipe.wait_for_frames().get_depth_frame().as_depth_frame()
# pixel_distance_in_meters = dpt_frame.get_distance(x,y)
# Get the depth for a certain pixel
x_pixel = 230
y_pixel = 230
print((colorized_depth[x_pixel][y_pixel]))
dist = aligned_depth_frame.get_distance(x_pixel, y_pixel)
print(dist)
depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_pixel, y_pixel], dist)
print(depth_point_in_meters_camera_coords)