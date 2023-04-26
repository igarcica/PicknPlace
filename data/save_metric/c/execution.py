import os
import subprocess
import csv

## Save trial tags intro a csv
data_filename = "./o1_gr_z_tags.csv"
my_file = open(data_filename, "wb")
wr = csv.writer(my_file, quoting=csv.QUOTE_NONNUMERIC)

## Publish pcd files as pointcloud topic during 8seg 
print("Reading PCD files") 
directory = './'
for filename in sorted(os.listdir(directory)):
    f = os.path.join(directory, filename)
    if os.path.isfile(f) and filename.endswith('.pcd'):
        print(filename)
        command = "timeout 8s rosrun pcl_ros pcd_to_pointcloud " + filename +" 0.1"
        print(command)
        os.system(command)
        wr.writerow((filename,''))






## REFERENCES
# https://linuxize.com/post/timeout-command-in-linux/
