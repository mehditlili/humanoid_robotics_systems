#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <iostream>

using namespace cv;
using namespace std;

bool roi_available = false;
Point pt1, pt2;
Mat cap_img;
Mat roi;
std::vector<Mat> hsv_channels;
Mat h_image, s_image;
Mat hist;
int histSize[] = {9};
// hue varies from 0 to 179, see cvtColor
float hranges[] = { 0, 180 };
const float* ranges[] = { hranges };
int channels[] = { 0 };
int dims = 1;
int nimages = 1;
Rect tracking_window;

Mat get_hue_thres(Mat img)
{
     Mat img_hsv;
     std::vector<Mat> hsv_channels;
     cvtColor(img, img_hsv,CV_BGR2HSV);
     cv::split(img_hsv, hsv_channels);
     h_image = hsv_channels[0];
     Mat h_image_thres;
     //threshold saturation matrix
     threshold(h_image, h_image_thres, 10, 255, 0);
     return h_image_thres;
}

void mouse_click(int event, int x, int y, int flags, void *param)
{

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
        {
            std::cout<<"Mouse got_roiPressed"<<std::endl;
                pt1.x = x;
                pt1.y = y;
        break;
        }
        case CV_EVENT_LBUTTONUP:
        {
            std::cout<<"Mouse LBUTTON Released"<<std::endl;
            pt2.x = x;
            pt2.y = y;
            Mat roi_tmp(cap_img, Rect(pt1, pt2));
            roi = roi_tmp;
            std::cout<<"PT1"<<pt1.x<<", "<<pt1.y<<std::endl;
            std::cout<<"PT2"<<pt2.x<<","<<pt2.y<<std::endl;
            imshow("roi",roi);
            roi_available = true;

            // Part 4 starts here
            Mat h_image_thres = get_hue_thres(roi);
            imshow("saturaton", h_image_thres);
            calcHist(&h_image_thres, nimages, channels, Mat(), // do not use mask
                     hist, dims, histSize, ranges,
                     true,
                     false); // the histogram is uniform);
            normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );
            tracking_window = Rect(pt1, pt2);
            // Part 4 ends here


        break;
        }
    }
}


int main(int argc, char* argv[])
{

    VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
    namedWindow("webcam",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    cvSetMouseCallback("webcam", mouse_click, 0);


    while(1)
    {
        cap >> cap_img;
        if (cap_img.empty()) //if not success, break loop
        {
           cout << "Cannot read the frame from webcam" << endl;
           break;
        }

        // Part 5 starts here
        Mat back_projection;
        Mat cap_img_hsv = get_hue_thres(cap_img);
        if (roi_available)
        {
            calcBackProject(&cap_img_hsv, nimages, channels, hist, back_projection, ranges);
            imshow("backproject", back_projection);
            // Part 5 ends here

            // Part 6 starts her
            meanShift(back_projection, tracking_window, TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
            rectangle(cap_img, tracking_window, Scalar( 0, 255, 255 ), 2, 8, 0);
            // Part 6 ends here
        }


        imshow("webcam", cap_img);
        if(waitKey(10) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;

}
