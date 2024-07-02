rosrun image_view image_saver image:=/ext_camera/color/image_raw _filename_format:="z_c_%04i.png"

mkdir img
mv z_c_0001.png ./img
rm *.png *.ini

rosbag record /ext_camera/depth/image_rect_raw /ext_camera/depth/color/points --duration=0.1

rosrun pcl_ros pointcloud_to_pcd input:=/ext_camera/depth/color/points _prefix:=z_
rosrun pcl_ros pointcloud_to_pcd input:=/segment_table/pick _prefix:=segm_pick_
rosrun pcl_ros pointcloud_to_pcd input:=/segment_table/place _prefix:=segm_place_

mv ./img/*.png ./
rm -rf ./img
