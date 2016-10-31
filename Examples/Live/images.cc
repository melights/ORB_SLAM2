// Include standard headers
#include <stdio.h>
#include <vector>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <System.h>

bool playback = true;
int main(void)
{
    ORB_SLAM2::System SLAM("../../Vocabulary/ORBvoc.bin", "./image.yaml", ORB_SLAM2::System::MONOCULAR);
    vector<cv::Mat> images;
    cv::Mat image, R, Tvec, Rvec;
    char filename[500];
    for (int i = 1; i <= 900; i++)
    {
        sprintf(filename, "/home/long/data/digest_size/%04d.png", i);
        std::cout << filename << std::endl;
        image = cv::imread(filename, 1);
        images.push_back(image.clone());
    }
    int imageNum = 0;
    int adder = 1;
    fstream fout;
    fout.open("/home/long/output_traj_size.txt", ios::trunc | ios::out);
    while (1)
    {
        if (imageNum == 899)
            adder = -1;
        else if (imageNum == 0)
            adder = 1;
        imageNum += adder;
        image = images[imageNum];
        cv::Mat CameraPose = SLAM.TrackMonocular(image, 1);
        if(CameraPose.size[0]==0)
            continue;
        CameraPose.rowRange(0, 3).colRange(0, 3).copyTo(R);
        CameraPose.rowRange(0, 3).col(3).copyTo(Tvec);
        cv::Rodrigues(R, Rvec);
        if (adder == 1)
        {
            fout << Tvec.at<float>(0) << "\t" << Tvec.at<float>(1) << "\t" << Tvec.at<float>(2) << "\t" << Rvec.at<float>(0) << "\t" << Rvec.at<float>(1) << "\t" << Rvec.at<float>(2) << "\n";
        }

        cv::waitKey(20);
    }

    return 0;
}
