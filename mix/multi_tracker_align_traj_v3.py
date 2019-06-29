import sys
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
from mpl_toolkits.mplot3d import Axes3D
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
from inspect import currentframe, getframeinfo
import time
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path

from random import randint

import sys


def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)
    # sys.stdout.write('Loading video: ')
    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush() # As suggested by Rom Ruben (see: http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
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
    height = 480
    width = 640
    # height = 720
    # width = 1280
    # //cfg.enable_stream(RS2_STREAM_COLOR); //this will cause seg fault in Mat(...)
    # cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30) 
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30) 
    pipe.start(cfg)
    # pipe.start(cfg)

    # frameset = pipe.wait_for_frames()
    # pipe.stop()
    align = rs.align(rs.stream.color)
    # frameset = align.process(frameset)
    # color_frame = frameset.get_color_frame()
    # # depth_frame = frameset.get_depth_frame()
    # np_color_frame = np.asanyarray(color_frame.get_data())


    # Create a video cap for opencv
    number_frame = 0
    number_frame_max = 410
    # number_frame_max = 50
    
    video = []
    # print(np_color_frame)
    
    while number_frame<=number_frame_max:
        frameset = pipe.wait_for_frames()
        frameset.keep()
        color_frame = frameset.get_color_frame()
        # if not color_frame: 
        #     continue 
        np_color_frame = np.asanyarray(color_frame.get_data())
        # video.append(np.asanyarray(color_frame.get_data()))
        # print(np_color_frame)
        video.append(np_color_frame)
        # cv2.imshow('RealSense', np_color_frame)
        # cv2.waitKey(1)
        # video[number_frame]=np_color_frame
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', np_color_frame)  
        # print(int(number_frame/number_frame_max*100), "%")
        # print()
        progress(number_frame, number_frame_max, status='')
        number_frame += 1
        # time.sleep( 0.53333333333)
    
    # print(video)

    ## Select boxes
    bboxes = []
    colors = [] 
    
    while True:
        # draw bounding boxes over objects
        # selectROI's default behaviour is to draw box starting from the center
        # when fromCenter is set to false, you can draw box starting from top left corner
        bbox = cv2.selectROI('MultiTracker', video[0])
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
    # if tracker_type == 'BOOSTING':
    #     tracker = cv2.TrackerBoosting_create()
    # if tracker_type == 'MIL':
    #     tracker = cv2.TrackerMIL_create()
    # if tracker_type == 'KCF':
    #     tracker = cv2.TrackerKCF_create()
    # if tracker_type == 'TLD':
    #     tracker = cv2.TrackerTLD_create()
    # if tracker_type == 'MEDIANFLOW':
    #     tracker = cv2.TrackerMedianFlow_create()
    # if tracker_type == 'GOTURN':
    #     tracker = cv2.TrackerGOTURN_create()
    # if tracker_type == 'CSRT':
    #     tracker = cv2.TrackerCSRT_create()
    # cv2.TrackerMOSSE_create()
    for bbox in bboxes:
        multiTracker.add(cv2.TrackerCSRT_create(), video[0], bbox)
        # MOSSE

    # cv2.
    # print("Waiting...")


    #
    # while True:
    number_frame = 0
    list_x_pixel_object = []
    list_y_pixel_object = []
    list_x_pixel_pusher = []
    list_y_pixel_pusher = []
    pipe.stop()
    print("Start with tracking")
    # for i in range(1, len(video)-1):
    while number_frame <= len(video)-1:
        if verbose >= 1:
            frameinfo = getframeinfo(currentframe())
            print("Start loop: ", frameinfo.lineno)
        
        # Read a new frame
    
        # frameset = pipe.wait_for_frames()
        # #align frame set
        # frameset = align.process(frameset)
        # #get frame
        # color_frame = frameset.get_color_frame()
        # depth_frame = frameset.get_depth_frame()
        # np_color_frame = np.asanyarray(color_frame.get_data())
        # print(np_color_frame)

        # # Start timer
        # timer = cv2.getTickCount()
        
        # get updated location of objects in subsequent frames
        success, boxes = multiTracker.update(video[number_frame])
        # print(boxes[0])#, ", ", boxes[1][1])
        # if not success:
        #     break
        
        if success:
            # draw tracked objects
            for i, newbox in enumerate(boxes):
                p1 = (int(newbox[0]), int(newbox[1]))
                p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                cv2.rectangle(video[number_frame], p1, p2, colors[i], 2, 1)
                # print(p1, ", ", p2)
                # center = ()
                # center = (int(newbox[0] + newbox[2]/2), int(newbox[1] + newbox[3]/2))
                x_pixel = int(newbox[0] + newbox[2]/2)
                y_pixel = int(newbox[1] + newbox[3]/2)
                
                # dist = depth_frame.get_distance(x_pixel, y_pixel)
                # # print(dist)
                # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                # depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_pixel, y_pixel], dist)

                # Object
                if i == 0:
                    list_x_pixel_object.append(x_pixel)
                    list_y_pixel_object.append(y_pixel)
                #     trajectory_object_x = np.append(trajectory_object_x, [depth_point_in_meters_camera_coords[0]], axis=0)
                #     trajectory_object_y = np.append(trajectory_object_y, [depth_point_in_meters_camera_coords[1]], axis=0)
                #     trajectory_object_z = np.append(trajectory_object_z, [depth_point_in_meters_camera_coords[2]], axis=0)

                # Hand
                if i == 1:
                    list_x_pixel_pusher.append(x_pixel)
                    list_y_pixel_pusher.append(y_pixel)
                #     trajectory_pusher_x = np.append(trajectory_pusher_x, [depth_point_in_meters_camera_coords[0]], axis=0)
                #     trajectory_pusher_y = np.append(trajectory_pusher_y, [depth_point_in_meters_camera_coords[1]], axis=0)
                #     trajectory_pusher_z = np.append(trajectory_pusher_z, [depth_point_in_meters_camera_coords[2]], axis=0)

        else :
            # Tracking failure
            # cv2.putText(frame, "Tracking failure detected", (10,160), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            print("ERROR: Tracking failure")

        # Display result
        # show frame
        cv2.imshow('MultiTracker', video[number_frame])            
        
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

        # Iteration
        number_frame += 1
    print("Finished tracking part")

    trajectory_object = np.array([0.0, 0.0, 0.0])#np.zeros(1, 3)
    trajectory_object = np.append([trajectory_object], [[0.0, 0.0, 0.0]], axis=0)
    trajectory_pusher = np.array([0.0, 0.0, 0.0])#np.zeros(1, 3)
    trajectory_pusher = np.append([trajectory_pusher], [[0.0, 0.0, 0.0]], axis=0)
    list_trajectory_object = []
    list_trajectory_pusher = []
    print(trajectory_object)
    pipe.start(cfg)
    number_frame = 0
    while number_frame<=number_frame_max-1:
        frameset = pipe.wait_for_frames()
        # frameset.keep()
        #align frame set
        # print(frameset)
        frameset = align.process(frameset)
        #get frame
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()


        dist_obj = depth_frame.get_distance(list_x_pixel_object[number_frame], list_y_pixel_object[number_frame])
        # print(dist_obj)
        dist_pusher = depth_frame.get_distance(list_x_pixel_pusher[number_frame], list_y_pixel_pusher[number_frame])
        # print(dist_pusher)
        # print(dist)
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # print("object")
        depth_point_in_meters_camera_coords_object = rs.rs2_deproject_pixel_to_point(depth_intrin, [list_x_pixel_object[number_frame], list_y_pixel_object[number_frame]], dist_obj)
        # print(depth_point_in_meters_camera_coords_object)
        depth_point_in_meters_camera_coords_pusher = rs.rs2_deproject_pixel_to_point(depth_intrin, [list_x_pixel_pusher[number_frame], list_y_pixel_pusher[number_frame]], dist_pusher)
        
        # print(depth_point_in_meters_camera_coords_pusher)
        # print("trajectory: ",trajectory_object)
        # print("dept_vect: ",depth_point_in_meters_camera_coords_object)
        
        if depth_point_in_meters_camera_coords_object != [0, 0, 0] and depth_point_in_meters_camera_coords_pusher != [0, 0, 0]:# and i == 0:
        #     trajectory_object_x = np.append(trajectory_object_x, [depth_point_in_meters_camera_coords_object[0]], axis=0)
        #     trajectory_object_y = np.append(trajectory_object_y, [depth_point_in_meters_camera_coords_object[1]], axis=0)
        #     trajectory_object_z = np.append(trajectory_object_z, [depth_point_in_meters_camera_coords_object[2]], axis=0)


            trajectory_object = np.append(trajectory_object, [depth_point_in_meters_camera_coords_object], axis=0)
            trajectory_pusher = np.append(trajectory_pusher, [depth_point_in_meters_camera_coords_pusher], axis=0)
            # time.sleep(1)

            # list_trajectory_object.append(depth_point_in_meters_camera_coords_object)
            # list_trajectory_pusher.append(depth_point_in_meters_camera_coords_pusher)
        # # if depth_point_in_meters_camera_coords != [0, 0, 0]:# and i == 1:
        #     trajectory_pusher_x = np.append(trajectory_pusher_x, [depth_point_in_meters_camera_coords_pusher[0]], axis=0)
        #     trajectory_pusher_y = np.append(trajectory_pusher_y, [depth_point_in_meters_camera_coords_pusher[1]], axis=0)
        #     trajectory_pusher_z = np.append(trajectory_pusher_z, [depth_point_in_meters_camera_coords_pusher[2]], axis=0)



        # np_color_frame = np.asanyarray(color_frame.get_data())
        # video[number_frame]=np_color_frame
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', np_color_frame)  
        # print(int(number_frame/number_frame_max*100), "%")
        progress(number_frame, number_frame_max, status='')

        # Exit if ESC pressed
        # k = cv2.waitKey(1) & 0xff
        # if k == 27 : break

        number_frame += 1
    pipe.stop()
    print("Finished trajectory analysis part")
    # end of the loop
    # pipe.stop()
    # Plotting the trajectories
    trajectory_object = np.delete(trajectory_object, 0, 0)
    trajectory_object = np.delete(trajectory_object, 0, 0)
    # trajectory_object_y = np.delete(trajectory_object_y, 0, 0)
    # trajectory_object_z = np.delete(trajectory_object_z, 0, 0)

    # trajectory_pusher_x = np.delete(trajectory_pusher_x, 0, 0)
    # trajectory_pusher_y = np.delete(trajectory_pusher_y, 0, 0)
    trajectory_pusher = np.delete(trajectory_pusher, 0, 0)
    trajectory_pusher = np.delete(trajectory_pusher, 0, 0)
    # print(trajectory_pusher)
    # print(trajectory_object)
    # print(trajectory_object[0][1])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #for np_.. list
    ax.plot(trajectory_object[:, 0], trajectory_object[:, 1], trajectory_object[:, 2], 'r')
    ax.plot(trajectory_pusher[:, 0], trajectory_pusher[:, 1], trajectory_pusher[:, 2], 'b')
    ax.set_xlabel('X-direction [m]')
    ax.set_ylabel('Y-direction [m]')
    ax.set_zlabel('Z-direction [m]')

    plt.show()

if __name__ == '__main__' :
    main()
    # try:
    #    main()
    # # except :
    # #     print("ERROR: execption caught!")
    # #     pipe.stop()
    # finally:
    #     # pipe.stop()
    #     print("End of the script")
    #     # Stop streaming
    #     # pipe.stop()
        