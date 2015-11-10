#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
using namespace cv;
using namespace std;

bool roi_available = false;
Point pt1, pt2;

Mat img;
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

// returns a thresholded saturation image
Mat get_sat_thres(Mat img)
{
     Mat img_hsv;
     std::vector<Mat> hsv_channels;
     cvtColor(img, img_hsv,CV_BGR2HSV);
     cv::split(img_hsv, hsv_channels);
     s_image = hsv_channels[1];
     Mat s_image_thres;
     //threshold saturation matrix
     threshold(s_image, s_image_thres, 127, 255, 0);
     return s_image_thres;
}

void mouse_click(int event, int x, int y, int flags, void *param)
{

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
        {
            std::cout<<"Mouse LBUTTON Pressed"<<std::endl;
                pt1.x = x;
                pt1.y = y;
        break;
        }
        case CV_EVENT_LBUTTONUP:
        {
            std::cout<<"Mouse LBUTTON Released"<<std::endl;
            pt2.x = x;
            pt2.y = y;
            Mat roi_tmp(img, Rect(pt1, pt2));
            roi = roi_tmp;
            std::cout<<"PT1"<<pt1.x<<", "<<pt1.y<<std::endl;
            std::cout<<"PT2"<<pt2.x<<","<<pt2.y<<std::endl;
            imshow("roi",roi);
            // part 2 starts here
            // No need to save image and reread it, just save it once it is selected with the mouse
            imwrite( "/home/hrs2015/catkin_ws/src/tutorial_4/images/roi_img.jpg", roi);
            roi_available = true;
            // part 2 ends here

            // Part 3 and 4 start here
            Mat s_image_thres = get_sat_thres(roi);
            Mat img_hsv;
            std::vector<Mat> hsv_channels;
            imshow("saturaton", s_image_thres);
            waitKey(1);
            cvtColor(roi, img_hsv, CV_BGR2HSV);
            cv::split(img_hsv, hsv_channels);
            calcHist(&hsv_channels[0], nimages, channels, s_image_thres,
                     hist, dims, histSize, ranges,
                     true,
                     false); // the histogram is uniform);
            normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );
            tracking_window = Rect(pt1, pt2);
            // Part 3 and 4 end here

        break;
        }
    }
}

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img = cv_ptr->image;
    // If roi is available proceed with tracking
    if (roi_available)
    {
        // Part 5 starts here
        Mat back_projection, cap_img_hsv, cap_image_sat_mask;
        //Mat cap_img_hsv = get_hue_thres(cap_img);
        std::vector<Mat> cap_hsv_channels;
        cvtColor(img, cap_img_hsv,CV_BGR2HSV);
            cv::split(cap_img_hsv, cap_hsv_channels);
        threshold(cap_hsv_channels[2], cap_image_sat_mask, 127, 255, 0);
        calcBackProject(&cap_img_hsv, nimages, channels, hist, back_projection, ranges);
        imshow("back_projection", back_projection);
        back_projection = back_projection & cap_image_sat_mask;
        // Part 5 ends here

        // Part 6 and 7 start here
        // uncomment either algorithm to use it
        meanShift(back_projection, tracking_window, TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
        //CamShift(back_projection, tracking_window, TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
        rectangle(img, tracking_window, Scalar( 0, 255, 255 ), 2, 8, 0);
        // Part 6 and 7 end here
    }
    // Show incoming image from nao
    imshow("camera_image", img);
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tutorial_4");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/nao_robot/camera/top/camera/image_raw", 1000, imageCB);
    //ros::Subscriber sub = n.subscribe("/camera/image_raw", 1000, imageCB);

    ROS_INFO("Node starting..");
    namedWindow("camera_image",1);
    cvSetMouseCallback("camera_image", mouse_click, 0);
    ros::spin();
    return 0;
}
