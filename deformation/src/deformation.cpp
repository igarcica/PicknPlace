//https://answers.ros.org/question/141741/calculate-depth-from-point-cloud-xyz/?answer=381085#post-id-381085
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        //get image dimension once
        static bool runOnce = true;
        if (runOnce){
            cout << "Image dimension (Row,Col): " << cv_ptr->image.rows << " x " << cv_ptr->image.cols << endl;
            runOnce = false;
        }

	double mid_x = cv_ptr->image.rows/2;
	double mid_y = cv_ptr->image.cols/2;
	std::cout << "Pixel: " << mid_x << ", " << mid_y <<std::endl;

        //get global max depth value
        double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        std::cout << "Max value: " << max << endl;

        //get global min depth value
        double min = 0.0;
        cv::minMaxLoc(cv_ptr->image, &min, &max, 0, 0);
        std::cout << "Min value: " << min << endl;

        //get depth value at a point
        float distanceVal = cv_ptr->image.at<float>(100, 100);
        std::cout << "Distance value: " << distanceVal << "m" << endl;
        distanceVal = cv_ptr->image.at<float>(240, 424);
        std::cout << "Distance value: " << distanceVal << "m" << endl;

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "Getting Image depth value!" << std::endl;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("ext_camera/depth/image_rect_raw", MY_ROS_QUEUE_SIZE, imgcb);

    ros::spin();

    std::cout << "Done" << std::endl;

    return 0;
}
