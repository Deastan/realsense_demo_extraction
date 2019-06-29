import sys
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
# import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
from inspect import currentframe, getframeinfo


print("Environment Ready")

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
if __name__ == '__main__' :

    verbose = 0
    # if verbose == 2:
    #     print("line: ", frameinfo.lineno)
    #     frameinfo = getframeinfo(currentframe())        
    #     print(frameinfo.filename, frameinfo.lineno)
    # # Setup:
    # pipe = rs.pipeline()
    # cfg = rs.config()
    # cfg.enable_device_from_file("test_mouse_6.bag")
    # profile = pipe.start(cfg)
    try:
        # Params:
        trajectory_x = np.zeros(1)
        trajectory_y = np.zeros(1)
        trajectory_z = np.zeros(1)
        # Set up tracker.
        # Instead of MIL, you can also use

        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_types[2]
        tracker_type = 'CSRT'
        # Shortcut the code
        minor_ver = 5
        if int(minor_ver) < 3:
            tracker = cv2.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING':
                tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                tracker = cv2.TrackerGOTURN_create()
            if tracker_type == 'CSRT':
                tracker = cv2.TrackerCSRT_create()

        
        # # Setup:
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device_from_file("test_mouse_4.bag")
        height = 480
        width = 640
        # height = 720
        # width = 1280
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30) 
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30) 
        profile = pipe.start(cfg)

        # Skip 5 first frames to give the Auto-Exposure time to adjust
        # for x in range(1):
        pipe.wait_for_frames()
        
        # Store next frameset for later processing:
        frameset = pipe.wait_for_frames()
        # align
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        # # Read video
        # video = cv2.VideoCapture("easy_resized.mp4")

        # # Exit if video not opened.
        # if not video.isOpened():
        #     print("Could not open video")
        #     sys.exit()

        # # Read first frame.
        # ok, frame = video.read()
        # if not ok:
        #     print('Cannot read video file')
        #     sys.exit()
        np_color_frame = np.asanyarray(color_frame.get_data())
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', np_color_frame)
        # cv2.waitKey(1000)

        frame = np_color_frame
        # Define an initial bounding box
        bbox = (287, 23, 86, 320)

        # Uncomment the line below to select a different bounding box
        bbox = cv2.selectROI(frame, True)

        # # Initialize tracker with first frame and bounding box
        ok = tracker.init(frame, bbox)
        # print("Waiting...")
        number_frame = 0
        while True:
            if verbose >= 1:
                frameinfo = getframeinfo(currentframe())
                print("Start loop: ", frameinfo.lineno)
            number_frame += 1
            # Read a new frame
            # ok, frame = video.read()
            # Store next frameset for later processing:
            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)

            frameset = pipe.wait_for_frames()
            #align frame set
            frameset = align.process(frameset)
            #get frame
            color_frame = frameset.get_color_frame()
            depth_frame = frameset.get_depth_frame()
            np_color_frame = np.asanyarray(color_frame.get_data())
            frame = np_color_frame

            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)

            if not ok:
                break
            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            # Start timer
            timer = cv2.getTickCount()
            if verbose >= 3:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            if verbose >= 3:
                frameinfo = getframeinfo(currentframe())
                print("Frame: ", frame)
                print("bbox: ", bbox)
                print("ok: ", ok)
                print("Number of the frame is: ", number_frame)
            # Update tracker
            ok, bbox = tracker.update(frame)
            if verbose >= 3:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            if verbose >= 3:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,55,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (10,160), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            # Display position 
            # cv2.putText(frame, "x: " +  str(bbox[0]) + ", y: " +  str(bbox[1]), (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
            # cv2.putText(frame, "x: " +  str(bbox[2]) + ", y: " +  str(bbox[3]), (10,120), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
            # cv2.putText(frame, "x: " +  str(bbox[0]+bbox[2]) + ", y: " +  str(bbox[1]+bbox[3]), (10,140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            if ok:
                # Add the align
                # Create alignment primitive with color as its target stream:
                # x_pixel = int(bbox[0])
                # y_pixel = int(bbox[1])

                x_pixel = int(bbox[0] + bbox[2]/2)
                y_pixel = int(bbox[1] + bbox[3]/2)

                # print((colorized_depth[x_pixel][y_pixel]))
                dist = depth_frame.get_distance(x_pixel, y_pixel)
                # print(dist)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_pixel, y_pixel], dist)
                # print(depth_point_in_meters_camera_coords)
                # print(trajectory_x)
                if depth_point_in_meters_camera_coords != [0, 0, 0]:
                    trajectory_x = np.append(trajectory_x, [depth_point_in_meters_camera_coords[0]], axis=0)
                    trajectory_y = np.append(trajectory_y, [depth_point_in_meters_camera_coords[1]], axis=0)
                    trajectory_z = np.append(trajectory_z, [depth_point_in_meters_camera_coords[2]], axis=0)
            # print(trajectory_z[-1])
            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            # Display result
            cv2.imshow("Tracking", frame)
            if verbose >= 2:
                frameinfo = getframeinfo(currentframe())
                print("line: ", frameinfo.lineno)
            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27 : break
            if verbose >= 1:
                frameinfo = getframeinfo(currentframe())
                print("End loop: ", frameinfo.lineno)
    # except :
    #     print()
    finally:
        # Stop streaming
        # pipe.stop()
        trajectory_x = np.delete(trajectory_x, 0, 0)
        trajectory_y = np.delete(trajectory_y, 0, 0)
        trajectory_z = np.delete(trajectory_z, 0, 0)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(trajectory_x, trajectory_y, trajectory_z)
        plt.show()