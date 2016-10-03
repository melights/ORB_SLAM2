// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include<System.h>
using namespace cv;
VideoCapture cap("/home/long/data/scale/left.avi");
//cv::VideoCapture cap(0);
bool playback =true;
int main( void )
{
    //VideoWriter out;
    //out.open("video.avi", VideoWriter::fourcc('S', 'V', 'Q', '3'), 30, Size(640*16/9-1, 480*16/9-1), true);
Mat frame_left;
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("../../Vocabulary/ORBvoc.bin", "./video.yaml", ORB_SLAM2::System::MONOCULAR);

double frame_rate = cap.get(CV_CAP_PROP_FPS);
  double frame_msec = 1000 / frame_rate;

  // Seek to the end of the video.
  cap.set(CV_CAP_PROP_POS_AVI_RATIO, 1);

  // Get video length (because we're at the end).
  double video_max_time = cap.get(CV_CAP_PROP_POS_MSEC);

  cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0);

double video_current_time = cap.get(CV_CAP_PROP_POS_MSEC);
double frame_msec1;

    while(1){
        if (playback){
           if (video_current_time>=video_max_time){
                frame_msec1=-frame_msec;
           }
           if (video_current_time<=0 ){
                frame_msec1=frame_msec;
           }
           video_current_time += frame_msec1;
           cap.set(CV_CAP_PROP_POS_MSEC, video_current_time);
        }
        cap.grab();
        // cap_right.grab();
        cap.retrieve(frame_left);
        // cap_right.retrieve(frame_right);
        //cap >> frame_left;
        //cap_right >> frame_right;

        //stereoRemap(frame_left, frame_right, frame_left_rectified, frame_right_rectified);

            Mat CameraPose = SLAM.TrackMonocular(frame_left, 1);


    } // Check if the ESC key was pressed or the window was closed



    return 0;
}

