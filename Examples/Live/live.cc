// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include<System.h>
using namespace cv;
//VideoCapture cap_left("/home/long/data/scale/left.avi");
 cv::VideoCapture cap_left(1);

int main( void )
{
    //VideoWriter out;
    //out.open("video.avi", VideoWriter::fourcc('S', 'V', 'Q', '3'), 30, Size(640*16/9-1, 480*16/9-1), true);
Mat frame_left;
    cap_left.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap_left.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("../../Vocabulary/ORBvoc.bin", "./handheld.yaml", ORB_SLAM2::System::MONOCULAR);



    while(1){

        cap_left.grab();
        // cap_right.grab();
        cap_left.retrieve(frame_left);
        // cap_right.retrieve(frame_right);
        //cap_left >> frame_left;
        //cap_right >> frame_right;

        //stereoRemap(frame_left, frame_right, frame_left_rectified, frame_right_rectified);

            Mat CameraPose = SLAM.TrackMonocular(frame_left, 1);


    } // Check if the ESC key was pressed or the window was closed



    return 0;
}

