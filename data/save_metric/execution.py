import os
import subprocess
 
directory = './'
for filename in sorted(os.listdir(directory)):
    f = os.path.join(directory, filename)
    if os.path.isfile(f) and filename.endswith('.pcd'):
        command = "timeout 10s rosrun pcl_ros pcd_to_pointcloud " + filename +" 0.1"
        os.system(command)
        print(command)
#        hola = subprocess.check_output(["rosrun", "pcl_ros", "pcd_to_pointcloud", filename, "0.1"], shell=True, timeout=500000000)


## TO DO
# Ejecutar archivos en orden - OK
# Cntrl C automatico - OK
# Timeout - OK


## REFERENCES
# https://linuxize.com/post/timeout-command-in-linux/
