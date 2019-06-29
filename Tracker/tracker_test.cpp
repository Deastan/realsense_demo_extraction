// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/feature.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/kalman_filters.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/onlineBoosting.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/onlineMIL.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/tldDataset.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/tracker.hpp
// -- Up-to-date: /usr/local/include/opencv4/opencv2/tracking/tracking.hpp
#include "opencv2/opencv.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
  std::cout << "Try to make it work2!" << std::endl;

  if(argc<2){
    std::cout<<
      " Usage: tracker_test <video_name>\n"
      " example:\n"
      " tracker_test easy_resized.mp4\n"
      << std::endl;
    return 0;
    //End of the program
  }

  // declares all required variables 
  cv::Rect2d roi; //know as bboxes
  cv::Mat frame;

  // Good Tracker and recommand from friends
  Ptr<Tracker> tracker = TrackerCSRT::create();

  // set input video
  std::string video = argv[1];
  cv::VideoCapture cap(video);


  // get bounding box
  cap >> frame;

  // ROI CONTAIN THE POSITION OF THE TRACKED OBJECT and init roi
  roi=selectROI("tracker", frame);
  //quit if ROI was not selected
  if(roi.width==0 || roi.height==0)
    return 0;
  // initialize the tracker
  tracker->init(frame,roi);
  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for ( ;; )
  {
    // get frame from the video
    cap >> frame;
    // stop the program if no more images
    if(frame.rows==0 || frame.cols==0)
      break;
    // update the tracking result
    tracker->update(frame,roi);
    // draw the tracked object
    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
    // putText(frame, roi)
    std::cout<<"Try to screen position: x: " << roi.x << ", y: " << roi.y << std::endl;
    // cv::rectangle()
    // show image with the tracked object
    imshow("tracker",frame);
    //quit on ESC button
    if(waitKey(1)==27)break;
  }

  std::cout << "End of the main" << std::endl;

  return 0;
}
