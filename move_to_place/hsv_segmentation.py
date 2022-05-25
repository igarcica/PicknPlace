# Sources:
# https://realpython.com/python-opencv-color-spaces/

import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors

import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server

class hsv_segm:
    def __init__(self):
#        self.img = cv2.imread('towel_segm.png')
        self.area_thres = 70000
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/ext_camera/color/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/hola",Image,queue_size=1)
#        self.result

    def segment(self, img):
        #towel = cv2.imread('towel_segm.png')
        towel = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        hsv_towel = cv2.cvtColor(towel, cv2.COLOR_RGB2HSV)
        #plt.imshow(towel) # BGR
        #plt.show() 
        #plt.imshow(towel) # RGB
        #plt.show()
        #plt.imshow(hsv_towel) # HSV
        #plt.show()
        
        ##3D projection
        #r, g, b = cv2.split(towel)
        #fig = plt.figure()
        #axis = fig.add_subplot(1, 1, 1, projection="3d")
        #pixel_colors = towel.reshape((np.shape(towel)[0]*np.shape(towel)[1], 3))
        #norm = colors.Normalize(vmin=-1.,vmax=1.)
        #norm.autoscale(pixel_colors)
        #pixel_colors = norm(pixel_colors).tolist()
        #axis.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker=".")
        #axis.set_xlabel("Red")
        #axis.set_ylabel("Green")
        #axis.set_zlabel("Blue")
        #plt.show()
        #
        #h, s, v = cv2.split(hsv_towel)
        #fig = plt.figure()
        #axis = fig.add_subplot(1, 1, 1, projection="3d")
        #
        #axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
        #axis.set_xlabel("Hue")
        #axis.set_ylabel("Saturation")
        #axis.set_zlabel("Value")
        #plt.show()
        
        
        ## HSV range
        #light_white = (71, 15, 0)
        #dark_white = (135, 255, 255)
        #light_white = (71, 30, 155)
        #dark_white = (100, 255, 255)

        light_white = (0,0,183) #real setup without robot
        dark_white = (255, 9, 255) #real setup without robot
        light_white = (0,0,193) #real setup without robot
        #dark_white = (255, 18, 255) #real setup without robot
        dark_white = (255, 40, 255) #real setup without robot
        edge_white = (135, 40, 255)
        
        mask = cv2.inRange(hsv_towel, light_white, dark_white)
        
        result = cv2.bitwise_and(towel, towel, mask=mask)
#        print("publishing..")
        self.pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        
#        plt.subplot(1, 2, 1)
#        plt.imshow(mask, cmap="gray")
#        plt.subplot(1, 2, 2)
#        plt.imshow(result)
#        plt.show()

        area = np.count_nonzero(mask)
        print(area)
        if(area > self.area_thres):
            print("place diagonally")
        else:
            print("dont place")
    
#    def plots(self):
 
    ###ROS
    def callback(self, data):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        print("hola")
        self.segment(img)
    
    
    #self.pub_success_image.publish(self.bridge.cv2_to_imgmsg(self.pic_success, "bgr8"))
    #    segment()
    


def main(): #args):
    segm = hsv_segm()
    rospy.init_node('segmentation', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
#    main(sys.argv)
    main()
