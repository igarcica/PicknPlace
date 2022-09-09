#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import rospkg
import statistics
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from image_difference.cfg import ParamsConfig

class image_converter:
    def __init__(self):
        print("hola")
        srv = Server(ParamsConfig, self.callbackConf)
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for image_difference
        self.path = rospack.get_path('image_difference')
        self.path = self.path+'/images/'
        self.pub_error = rospy.Publisher("/image_difference/error",Float64,queue_size=1)
        self.pub_pixelscurrent = rospy.Publisher("/image_difference/pixelscurrent",Int32,queue_size=1) 
        self.pub_pixelstotal = rospy.Publisher("/image_difference/pixelstotal",Int32,queue_size=1)
        self.pub_success_image = rospy.Publisher("/image_difference/successfull_placement_image",Image,queue_size=1)
        self.pub_current_image = rospy.Publisher("/image_difference/current_image",Image,queue_size=1)
        #self.pub_success_image_d = rospy.Publisher("/image_difference/successfull_placement_image_d",Image,queue_size=1)
        #self.pub_current_image_d = rospy.Publisher("/image_difference/current_image_d",Image,queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ext_camera/color/image_raw",Image,self.callback)
        self.depth_image_sub = rospy.Subscriber("/ext_camera/aligned_depth_to_color/image_raw",Image,self.callback_depth)

        self.success_num_pixels = 1.0
        self.take_pic_table = False
        self.success_place = False
        self.is_pic_success = False
        self.save_img = False
        self.threshold = 17
        self.kernel_x = 10
        self.kernel_y = 10
        self.dilate_iter = 6
        self.erode_iter = 8
   
        self.is_pic_table = False
        self.pic_success = Image()
       
        self.pic_table = Image()
        self.pic_table_d = Image()
        self.mask = 0 


    def callbackConf(self, config, level):
        rospy.loginfo("""Reconfigure Request: {take_picture_table}, {successful_placement}, {save_current_img}, {threshold}, {kernel_x}, {kernel_y}, {dilate_iter}, {erode_iter}""".format(**config))
        self.take_pic_table = config["take_picture_table"]
        self.success_place = config["successful_placement"]
        self.save_img = config["save_current_img"]
        self.threshold = config["threshold"]
        self.kernel_x = config["kernel_x"]
        self.kernel_y = config["kernel_y"]
        self.dilate_iter = config["dilate_iter"]
        self.erode_iter = config["erode_iter"]
        return config

    def nothing(x):
        pass

    def get_num_pixels(self, current_image):
        diff = cv2.absdiff(current_image,self.pic_table)
        #cv2.imshow("Diff", diff)
        #cv2.waitKey(3)
        diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Gray", diff)
        #cv2.waitKey(3)

        # Read the threshold slider
	ret, mask = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
        #cv2.imshow("Mask", mask)
        #cv2.waitKey(3)
	# Create the kernel
	kernel = np.ones((self.kernel_x,self.kernel_y),np.uint8)
	# Delete the noise
	mask = cv2.dilate(mask, kernel, iterations = self.dilate_iter)
	mask = cv2.erode( mask, kernel, iterations = self.erode_iter)

        # Find exterior contours
	cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	# Draw the contours
	cv2.drawContours(mask, cnts, -1, 255, -1)

        ## create a Mat like current_image
	imask = mask > 0
	canvas = np.zeros_like(current_image, np.uint8)
	canvas[imask] = current_image[imask]

        #Understand depth and color values
        vect = [np.sum(n>0) for n in current_image]
        np.savetxt('color_vect_im.txt', vect, fmt='%u')
        vect = [np.sum(n>0) for n in canvas]
        np.savetxt('color_vect_mask.txt', vect, fmt='%u')
        #np.savetxt('color_im.txt', current_image)
        #np.savetxt('color_mask.txt', canvas, fmt='%u')


        im = cv2.circle(current_image, (900,100), 10, (0,255, 0), 2)
        cv2.imshow("TEST", current_image)
        cv2.waitKey(3)
        ##current_image.size
	test = np.zeros_like(current_image)
#        fil = open('file', 'w')
#        pixel = im.load()
#        row, column = im.size
#        for y in range(column):
#            for x in range(row):
#                pixel = pix[x, y]
#                fil.write(str(pixel) + '\n')
#        fil.close()
        
        self.mask = imask
        print "Color: ", self.mask[100,900]

        return (float(cv2.countNonZero(mask)), canvas)


    def callback(self,data):
        try:
            int_blur = 21
            if self.take_pic_table and not self.is_pic_table:
                self.pic_table = self.bridge.imgmsg_to_cv2(data,"bgr8")
                #cv2.imshow("Pic Table", self.pic_table)
                #cv2.waitKey(3)
                self.pic_table = cv2.GaussianBlur(self.pic_table, (int_blur,int_blur), cv2.BORDER_DEFAULT)
                #cv2.imshow("Pic table Gaussian", self.pic_table)
                #cv2.waitKey(3)
                self.is_pic_table = True
            if not self.take_pic_table and self.is_pic_table:
                self.is_pic_table = False

            if self.take_pic_table and not self.success_place:
                self.pic_success = self.bridge.imgmsg_to_cv2(data,"bgr8")
                #cv2.imshow("Pic success", self.pic_success)
                #cv2.waitKey(3)
                self.pic_success = cv2.GaussianBlur(self.pic_success, (int_blur,int_blur), cv2.BORDER_DEFAULT)
                #cv2.imshow("Pic success gaussian", self.pic_success)
                #cv2.waitKey(3)
                self.success_num_pixels, self.pic_success = self.get_num_pixels(self.pic_success)
                #cv2.imshow("Pic success get num", self.pic_success)
                #cv2.waitKey(3)
                self.pub_success_image.publish(self.bridge.cv2_to_imgmsg(self.pic_success, "bgr8")) 

            if self.is_pic_table and self.success_place and not self.is_pic_success: 
                self.is_pic_success = True
            if not self.success_place and self.is_pic_success:
                self.is_pic_success = False


            if self.is_pic_table and self.is_pic_success:
                cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                image_blur = cv2.GaussianBlur(cv_image, (int_blur,int_blur), cv2.BORDER_DEFAULT)

                current_num_pixels, canvas = self.get_num_pixels(image_blur)
                cv2.imshow("Current pic get num", canvas)
                cv2.waitKey(3)
                error = float(current_num_pixels/self.success_num_pixels)
		if error > 1: 
		    error = 1.0

                self.pub_pixelscurrent.publish(int(current_num_pixels))
                self.pub_pixelstotal.publish(int(self.success_num_pixels))
		self.pub_error.publish(error)
                self.pub_success_image.publish(self.bridge.cv2_to_imgmsg(self.pic_success, "bgr8"))
                self.pub_current_image.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
            
            if self.save_img:
                cv2.imwrite('current_img.png', canvas)
            
        except CvBridgeError as e:
            print(e)

    def callback_depth(self,data):
        if self.success_place:
            pic_depth = self.bridge.imgmsg_to_cv2(data)#,"16UC1")
            #cv2.imshow("Pic Depth", pic_depth) #Does not work because it interprets the depth values as black/white 0-255 (?). What makes the image all black except the closer points
            #cv2.waitKey(3)
            canvas = np.zeros_like(pic_depth, np.uint16) #Create a black mask
	    canvas[self.mask] = pic_depth[self.mask] #Save the points where there is no mask (mask=True)
            self.segmented_depth = canvas

            print "Depth canvas: ", canvas[100,900]
            print "Depth: ", pic_depth[100,900]
            vect = [np.sum(n>0) for n in pic_depth]
            np.savetxt('depth_vec_im.txt', vect, fmt='%u')
            vect = [np.sum(n>0) for n in canvas]
            np.savetxt('depth_vecc_mask.txt', vect, fmt='%u')
            np.savetxt('depth_im.txt', pic_depth)
            np.savetxt('depth_mask.txt', canvas, fmt='%u')

            print "Depth canvas: ", canvas[100,900]
            print "Depth: ", pic_depth[100,900]
            cv2.imshow("Pic depth", pic_depth)
            cv2.waitKey(3)
            cv2.imshow("Pic depth MASK", canvas)
            cv2.waitKey(3)

            #print "flatten: ", canvas.flatten()
            #print "mean: ", statistics.mean(canvas.flatten())

            nonzero = canvas[canvas>1]
            print "nonzer: ", nonzero
            np.savetxt('nonzero.txt', nonzero)
            print "mean: ", np.mean(nonzero)

#            # https://stackoverflow.com/questions/47751323/get-depth-image-in-grayscale-in-ros-with-imgmsg-to-cv2-python
#            # The depth image is a single-channel float32 image
#            # the values is the distance in mm in z axis
#            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
#            # Convert the depth image to a Numpy array since most cv2 functions
#            # require Numpy arrays.
#            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
#            # Normalize the depth image to fall between 0 (black) and 1 (white)
#            # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
#            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
#            # Resize to the desired size
#            #cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
#            #self.depthimg = cv_image_resized
#            cv2.imshow("Image from my node", cv_image_norm)
#            cv2.waitKey(1)
#            print "new: ", cv_image_norm[100,900]

def compute_mean():
    print "hola"


def main():
    rospy.init_node('image_difference', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__=='__main__':
    main()
