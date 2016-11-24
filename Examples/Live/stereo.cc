// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <System.h>

bool playback = true;
int main(void)
{
    ORB_SLAM2::System SLAM("../../Vocabulary/ORBvoc.bin", "./digest_stereo.yaml", ORB_SLAM2::System::STEREO);

    cv::Mat imageL,imageR;
    char filenameL[500],filenameR[500];

    int imageNum = 1;
    int adder = 1;
    while (1)
    {
        if (imageNum == 900)
            adder = -1;
        else if (imageNum == 1)
            adder = 1;
        sprintf(filenameL, "/home/long/data/digest_stereo/%04d_L.png", imageNum);
        sprintf(filenameR, "/home/long/data/digest_stereo/%04d_R.png", imageNum);
        //std::cout << filenameL << std::endl;
        imageL = cv::imread(filenameL, 1);
        imageR = cv::imread(filenameR, 1);
        imageNum += adder;
        
        SLAM.TrackStereo(imageL,imageR,1);        
        cv::imshow("aa",imageL);
        cv::waitKey();
    }

    return 0;
}
