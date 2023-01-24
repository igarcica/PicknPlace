rosrun image_view image_saver image:=/ext_camera/color/image_raw _filename_format:="z_c_%04i.png"
rosrun image_view image_saver image:=/lateral_camera/color/image_raw _filename_format:="l_c_%04i.png"
rosrun image_view image_saver image:=/frontal_camera/color/image_raw _filename_format:="f_c_%04i.png"

mkdir img
mv f_c_0001.png ./img
mv l_c_0001.png ./img
mv z_c_0001.png ./img
rm *.png *.ini

rosbag record /ext_camera/depth/image_rect_raw /lateral_camera/depth/image_rect_raw /frontal_camera/depth/image_rect_raw --duration=0.1

rosrun pcl_ros pointcloud_to_pcd input:=/ext_camera/depth/color/points _prefix:=z_
rosrun pcl_ros pointcloud_to_pcd input:=/frontal_camera/depth/color/points _prefix:=f_
rosrun pcl_ros pointcloud_to_pcd input:=/lateral_camera/depth/color/points _prefix:=l_

mv ./img/*.png ./
rm -rf ./img
