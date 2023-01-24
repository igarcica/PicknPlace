#!/usr/bin/env python

import rospy
import csv
from visualization_msgs.msg import MarkerArray

#Como se cuando ha cambiado de pointcloud?

data_filename = "./o1_e1-12_gr_z.csv"
my_file = open(data_filename, "wb")
wr = csv.writer(my_file, delimiter=",")


def save_mean_values(mean_data):
    print("Writing mean values...")
    print(mean_data)
    wr.writerow(mean_data)
    print("hola")

def callback(data):
    global prev_mean
    mean1=data.markers[2].text
    mean2=data.markers[3].text
    mean3=data.markers[4].text
    mean4=data.markers[5].text
    print(mean1)
    print(data.markers[2].text)
    print("Means: "+mean1+" / "+mean2+" / "+mean3+" / "+mean4)

    if mean1 != prev_mean:
        print("DIFERENTE")
        prev_mean=mean1
        #Save data
        means=[mean1, mean2, mean3, mean4]
        save_mean_values(means)

def listener():
    global prev_mean

    print("Saving metric data")
    prev_mean = 0

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/segment_table/corners', MarkerArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

