import sys
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
from mpl_toolkits.mplot3d import Axes3D
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
from inspect import currentframe, getframeinfo

# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path

from random import randint


# (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
# print(major_ver)

def main():
    # Create object for parsing command-line options
    parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                    Remember to change the stream resolution, fps and format to match the recorded.")
    # Add argument which takes path to a bag file as an input
    parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
    # Parse the command line arguments to an object
    args = parser.parse_args()
    # Safety if no parameter have been given
    if not args.input:
        print("No input paramater have been given.")
        print("For help type --help")
        exit()
    # Check if the given file have bag extension
    if os.path.splitext(args.input)[1] != ".bag":
        print("The given file is not of correct file format.")
        print("Only .bag files are accepted")
        exit()


    verbose = 0

    # Save data for to plot the trajectory
    trajectory_object_x = np.zeros(1)
    trajectory_object_y = np.zeros(1)
    trajectory_object_z = np.zeros(1)

    trajectory_pusher_x = np.zeros(1)
    trajectory_pusher_y = np.zeros(1)
    trajectory_pusher_z = np.zeros(1)

    # # Setup:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(str(args.input))
    # height = 480
    # width = 640
    height = 720
    width = 1280
    # //cfg.enable_stream(RS2_STREAM_COLOR); //this will cause seg fault in Mat(...)
    # cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30) 
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30) 

    pipe.start(cfg)
    frameset = pipe.wait_for_frames()
    pipe.stop()

    align = rs.align(rs.stream.color)
    # frameset = align.process(frameset)
    color_frame = frameset.get_color_frame()
    # depth_frame = frameset.get_depth_frame()
    np_color_frame = np.asanyarray(color_frame.get_data())

    ## Select boxes
    bboxes = []
    colors = [] 
    
    while True:
        # draw bounding boxes over objects
        # selectROI's default behaviour is to draw box starting from the center
        # when fromCenter is set to false, you can draw box starting from top left corner
        bbox = cv2.selectROI('MultiTracker', np_color_frame)
        bboxes.append(bbox)
        colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))
        print("Press q to quit selecting boxes and start tracking")
        print("Press any other key to select next object")
        k = cv2.waitKey(0) & 0xFF
        if (k == 113):  # q is pressed
            break
    
    print('Selected bounding boxes {}'.format(bboxes))

    # Create MultiTracker object
    multiTracker = cv2.MultiTracker_create()
    
    # Initialize MultiTracker 
    for bbox in bboxes:
        multiTracker.add(cv2.TrackerCSRT_create(), np_color_frame, bbox)

    # cv2.
    # print("Waiting...")
    number_frame = 0
    # pipe.stop()
    pipe.start(cfg)

    while number_frame<1000: #True:
        if verbose >= 1:
            frameinfo = getframeinfo(currentframe())
            print("Start loop: ", frameinfo.lineno)
        number_frame += 1
        # Read a new frame
    
        # frameset = 
        #align frame set
        frameset = align.process(pipe.wait_for_frames())
        #get frame
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        np_color_frame = np.asanyarray(color_frame.get_data())
        # print(np_color_frame)

        # # Start timer
        # timer = cv2.getTickCount()
        
        # get updated location of objects in subsequent frames
        success, boxes = multiTracker.update(np_color_frame)
        # if not success:
        #     break
        
        if False:#success:
            # draw tracked objects
            for i, newbox in enumerate(boxes):
                p1 = (int(newbox[0]), int(newbox[1]))
                p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                cv2.rectangle(np_color_frame, p1, p2, colors[i], 2, 1)

                # center = ()
                # center = (int(newbox[0] + newbox[2]/2), int(newbox[1] + newbox[3]/2))
                x_pixel = int(newbox[0] + newbox[2]/2)
                y_pixel = int(newbox[1] + newbox[3]/2)
                dist = depth_frame.get_distance(x_pixel, y_pixel)
                # print(dist)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_pixel, y_pixel], dist)

                if i == 0 and depth_point_in_meters_camera_coords != [0, 0, 0]:
                    trajectory_object_x = np.append(trajectory_object_x, [depth_point_in_meters_camera_coords[0]], axis=0)
                    trajectory_object_y = np.append(trajectory_object_y, [depth_point_in_meters_camera_coords[1]], axis=0)
                    trajectory_object_z = np.append(trajectory_object_z, [depth_point_in_meters_camera_coords[2]], axis=0)
                if i == 1 and depth_point_in_meters_camera_coords != [0, 0, 0]:
                    trajectory_pusher_x = np.append(trajectory_pusher_x, [depth_point_in_meters_camera_coords[0]], axis=0)
                    trajectory_pusher_y = np.append(trajectory_pusher_y, [depth_point_in_meters_camera_coords[1]], axis=0)
                    trajectory_pusher_z = np.append(trajectory_pusher_z, [depth_point_in_meters_camera_coords[2]], axis=0)

        else :
            # Tracking failure
            # cv2.putText(frame, "Tracking failure detected", (10,160), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            print("ERROR: Tracking failure")

        # Display result
        # show frame
        cv2.imshow('MultiTracker', np_color_frame)            
        
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
    

    # end of the loop
    pipe.stop()
    # Plotting the trajectories
    trajectory_object_x = np.delete(trajectory_object_x, 0, 0)
    trajectory_object_y = np.delete(trajectory_object_y, 0, 0)
    trajectory_object_z = np.delete(trajectory_object_z, 0, 0)

    trajectory_pusher_x = np.delete(trajectory_pusher_x, 0, 0)
    trajectory_pusher_y = np.delete(trajectory_pusher_y, 0, 0)
    trajectory_pusher_z = np.delete(trajectory_pusher_z, 0, 0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_object_x, trajectory_object_y, trajectory_object_z, 'r')
    ax.plot(trajectory_pusher_x, trajectory_pusher_y, trajectory_pusher_z, 'b')
    ax.set_xlabel('X-direction [m]')
    ax.set_ylabel('Y-direction [m]')
    ax.set_zlabel('Z-direction [m]')

    plt.show()

if __name__ == '__main__' :
    try:
       main()
    # except :
    #     print("ERROR: execption caught!")
    #     pipe.stop()
    finally:
        # pipe.stop()
        print("End of the script")
        # Stop streaming
        # pipe.stop()
        