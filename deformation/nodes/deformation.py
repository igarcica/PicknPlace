#https://github.com/IntelRealSense/realsense-ros/issues/714
#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

def convert_depth_image(ros_image):
    bridge = CvBridge()
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
    #Convert the depth image using the default passthrough encoding
                depth_image = bridge.imgmsg_to_cv2(ros_image, "16UC1")# desired_encoding="passthrough")

    except CvBridgeError, e:
 	          print e
     #Convert the depth image to a Numpy array
    depth_array = np.array(depth_image, dtype=np.float32)
    #rospy.loginfo(depth_array)

    #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

    center_idx = np.array(depth_array.shape) / 2
    print ('center depth:', depth_array[center_idx[0], center_idx[1]])
    #print ('center depth:', depth_array[478, 846])

    #print "hola", center_idx[0], center_idx[1]
    im = cv2.circle(depth_array, (0,0), 100, (255, 0, 0), 2)
    #cv2.imshow("Image window", depth_image)
    cv2.imshow("Image window", depth_array)
    cv2.waitKey(3)

def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/ext_camera/depth/image_rect_raw", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
        print "hola"
	pixel2depth()


##include <iostream>
##include <ros/ros.h>
##include <sensor_msgs/Image.h>
##include <sensor_msgs/image_encodings.h>
##include <cv_bridge/cv_bridge.h>
#
##include <opencv2/highgui/highgui.hpp>
#
#using namespace std;
#
#static const uint32_t MY_ROS_QUEUE_SIZE = 1000;
#
#void imgcb(const sensor_msgs::Image::ConstPtr& msg)
#{
#    try {
#        cv_bridge::CvImageConstPtr cv_ptr;
#        cv_ptr = cv_bridge::toCvShare(msg);
#
#        //get image dimension once
#        static bool runOnce = true;
#        if (runOnce){
#            cout << "Image dimension (Row,Col): " << cv_ptr->image.rows << " x " << cv_ptr->image.cols << endl;
#            runOnce = false;
#        }
#
#        //get global max depth value
#        double max = 0.0;
#        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
#        std::cout << "Max value: " << max << endl;
#
#        //get global min depth value
#        double min = 0.0;
#        cv::minMaxLoc(cv_ptr->image, &min, &max, 0, 0);
#        std::cout << "Min value: " << min << endl;
#
#        //get depth value at a point
#        float distanceVal = cv_ptr->image.at<float>(100, 100);
#        std::cout << "Distance value: " << distanceVal << "m" << endl;
#
#    } catch (const cv_bridge::Exception& e) {
#        ROS_ERROR("cv_bridge exception: %s", e.what());
#    }
#}
#
#int main(int argc, char* argv[])
#{
#    ros::init(argc, argv, "foo");
#
#    std::cout << "Getting Image depth value!" << std::endl;
#
#    ros::NodeHandle nh;
#    ros::Subscriber sub = nh.subscribe("camera/depth/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
#
#    ros::spin();
#
#    std::cout << "Done" << std::endl;
#
#    return 0;
#}
