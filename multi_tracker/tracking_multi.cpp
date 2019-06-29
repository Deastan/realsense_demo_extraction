// https://stackoverflow.com/questions/52516513/get-the-x-and-y-pixel-co-ordinates-of-the-roi-that-has-been-tracked-in-a-trackin

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

// Convert to string
//#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

vector<string> trackerTypes = { "BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT" };

// create tracker by name
Ptr<Tracker> createTrackerByName(string trackerType)
{
    Ptr<Tracker> tracker;
    if (trackerType == trackerTypes[0])
        tracker = TrackerBoosting::create();
    else if (trackerType == trackerTypes[1])
        tracker = TrackerMIL::create();
    else if (trackerType == trackerTypes[2])
        tracker = TrackerKCF::create();
    else if (trackerType == trackerTypes[3])
        tracker = TrackerTLD::create();
    else if (trackerType == trackerTypes[4])
        tracker = TrackerMedianFlow::create();
    else if (trackerType == trackerTypes[5])
        tracker = TrackerGOTURN::create();
    else if (trackerType == trackerTypes[6])
        tracker = TrackerMOSSE::create();
    else if (trackerType == trackerTypes[7])
        tracker = TrackerCSRT::create();
    else {
        cout << "Incorrect tracker name" << endl;
        cout << "Available trackers are: " << endl;
        for (vector<string>::iterator it = trackerTypes.begin(); it != trackerTypes.end(); ++it)
            std::cout << " " << *it << endl;
    }
    return tracker;
}

// Fill the vector with random colors
void getRandomColors(vector<Scalar> &colors, int numColors)
{
    RNG rng(0);
    for (int i = 0; i < numColors; i++)
        colors.push_back(Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
}

int main(int argc, char * argv[])
{
    cout << "Available tracking algorithms are:" << endl;
    for (vector<string>::iterator it = trackerTypes.begin(); it != trackerTypes.end(); ++it)
        std::cout << " " << *it << endl;

    string trackerType = "CSRT";
    // string trackerType = "KCF";
    cout << "The Selected tracker is " << trackerType << endl;

    string videoPath = "easy_resized.mp4";

    // Initialize MultiTracker with tracking algo
    vector<Rect> bboxes;
    Mat frame;
    cv::VideoCapture cap(videoPath);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    if (!cap.isOpened())
    {
        cout << "Error opening video file " << videoPath << endl;
        return -1;
    }
    cap >> frame;

    bool showCrosshair = true;
    bool fromCenter = false;

    cv::selectROIs("MultiTracker", frame, bboxes, showCrosshair, fromCenter);

    if (bboxes.size() < 1)
        return 0;

    vector<Scalar> colors;
    getRandomColors(colors, bboxes.size());

    // Create multitracker
    Ptr<MultiTracker> multiTracker = cv::MultiTracker::create();

    // initialize multitracker
    for (int i = 0; i < bboxes.size(); i++)
        multiTracker->add(createTrackerByName(trackerType), frame, Rect2d(bboxes[i]));

    cout << "Started tracking, press ESC to quit." << endl;

    while (cap.isOpened())
    {
        cap >> frame;
        if (frame.empty()) break;

        //update the tracking result with new frame

        multiTracker->update(frame);

        putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

        // draw tracked objects
        for (unsigned i = 0; i < multiTracker->getObjects().size(); i++)
        {                                                                               
            rectangle(frame, multiTracker->getObjects()[i], colors[i], 2, 1);
            bboxes[i] = multiTracker->getObjects()[i];
        }

        cout << "\nPosition of box1 in X-axis :" << bboxes[0].x << endl;
        cout << "\nPosition of box1 in Y-axis :" << bboxes[0].y << endl;   
        cout << "\nPosition of box2 in X-axis :" << bboxes[1].x << endl;
        cout << "\nPosition of box2 in Y-axis :" << bboxes[1].y << endl;

        resize(frame, frame, Size(1280, 720), 0, 0, INTER_CUBIC);
        imshow("MultiTracker", frame);
        if (waitKey(1) == 27) break; 

    }



}