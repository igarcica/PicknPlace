/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// PAL headers
#include <vision_pick_place/pcl_filters.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

// Std C++ headers
#include <string>

namespace pal {

  class SegmentPlane {
  public:
    SegmentPlane(ros::NodeHandle& nh,
                 ros::NodeHandle& pnh);

    virtual ~SegmentPlane();

    void run();

  protected:

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    //void segmCloudCallback(const sensor_msgs::PointCloud2ConstPtr& segmCloud);
    void pickCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pickCloud);
    void placeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& placeCloud);
    void nonplaneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& nonplaneCloud);

    void start();
    void stop();

    void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& planeCloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nonPlaneCloud,
                 pcl::uint64_t& stamp,
                 const std::string& frameId);

    void publishEmptyClouds(pcl::uint64_t& stamp,
                            const std::string& frameId);

    void normalizeVector(std::vector<double>& v);

    ros::NodeHandle& _nh, _pnh;
    ros::CallbackQueue _cbQueue;
    bool _enabled;
    double _rate;

    tf::TransformListener _tfListener;

    // frame in which the point cloud will be transformed
    std::string _processingFrame;

    // pass-through filter parameters:
    std::string _axis, _axisPick, _axisPlace;
    double _min, _max, _minPick, _maxPick, _minPlace, _maxPlace;

    // downsampling filter parameters:
    double _downSamplingSize;

    // ROS subscribers
    ros::Subscriber _cloudSub;
    //ros::Subscriber _segmCloudSub;
    ros::Subscriber _pickCloudSub;
    ros::Subscriber _placeCloudSub;
    ros::Subscriber _nonplaneCloudSub;

    // ROS publishers
    ros::Publisher _planeCloudPub;
    ros::Publisher _nonPlaneCloudPub;
    ros::Publisher _mainPlanePosePub;
    ros::Publisher _pickCloudPub;
    ros::Publisher _placeCloudPub;

    ros::Publisher _convexCloudPub;
    ros::Publisher _cornersMarkersPub;
    ros::Publisher _graspPointPub;
    ros::Publisher _garmentEdgePub;

    //Variables globales:
    int n_frames;
    pcl::PointXYZ current_sum_c1;
    pcl::PointXYZ current_sum_c2;
    pcl::PointXYZ current_sum_c3;
    pcl::PointXYZ current_sum_c4;
    ros::Publisher _graspPointAnglePub;
  };

  SegmentPlane::SegmentPlane(ros::NodeHandle& nh,
                                     ros::NodeHandle& pnh):
    _nh(nh),
    _pnh(pnh),
    _enabled(false),
    _rate(1),
    _processingFrame(""),
    _axis(""),
    _min(0.5),
    _max(10),
    _axisPick(""),
    _minPick(0.5),
    _maxPick(10),
    _axisPlace(""),
    _minPlace(0.5),
    _maxPlace(10),
    _downSamplingSize(0.01)
  {
    _nh.setCallbackQueue(&_cbQueue);

    pnh.param<double>("rate", _rate, _rate);
    pnh.param<std::string>("frame", _processingFrame, _processingFrame);
    pnh.param<std::string>("passthrough_axis", _axis, _axis);
    pnh.param<double>("passthrough_min", _min, _min);
    pnh.param<double>("passthrough_max", _max, _max);
    pnh.param<std::string>("passthrough_axis_pick", _axisPick, _axisPick);
    pnh.param<double>("passthrough_min_pick", _minPick, _minPick);
    pnh.param<double>("passthrough_max_pick", _maxPick, _maxPick);
    pnh.param<std::string>("passthrough_axis_place", _axisPlace, _axisPlace);
    pnh.param<double>("passthrough_min_place", _minPlace, _minPlace);
    pnh.param<double>("passthrough_max_place", _maxPlace, _maxPlace);
    pnh.param<double>("downsampling_size", _downSamplingSize, _downSamplingSize);

    ROS_INFO_STREAM("The node will operate at maximum " << _rate << " Hz");

    if ( _processingFrame.empty() )
      ROS_INFO("The point cloud will be filtered in its original frame");
    else
      ROS_INFO_STREAM("The point cloud will be filtered after transforming it to the "
                      << _processingFrame << " frame");

    if ( _axis.empty() )
      ROS_INFO("The passthrough filter has been disabled");
    else
      ROS_INFO_STREAM("A passthrough filter on " << _axis <<
                      " axis will be applied with min: " << _min <<
                      " and max: " << _max);
    ROS_INFO_STREAM("Downsampling leaf size: " << _downSamplingSize << "");

    _planeCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane", 1);
    _nonPlaneCloudPub = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("nonplane", 1);
    _mainPlanePosePub = _pnh.advertise<geometry_msgs::PoseStamped>("plane_pose", 1);
    _pickCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("pick", 1);
    _placeCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("place", 1);

    _convexCloudPub   = _pnh.advertise< pcl::PointCloud<pcl::PointXYZ> >("convex", 1);
    _cornersMarkersPub = _pnh.advertise<visualization_msgs::MarkerArray>("corners", 1);
    _graspPointPub    = _pnh.advertise<visualization_msgs::Marker>("grasp_point", 1);

    n_frames=0;
    _graspPointAnglePub    = _pnh.advertise<std_msgs::Float64>("grasp_angle", 1);
    _garmentEdgePub   = _pnh.advertise<std_msgs::Float64>("garment_edge", 1);

  }

  SegmentPlane::~SegmentPlane()
  {
  }

  void SegmentPlane::publishEmptyClouds(pcl::uint64_t& stamp,
                                        const std::string& frameId)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyNonPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    publish(emptyPlaneCloud,
            emptyNonPlaneCloud,
            stamp,
            frameId);

  }

  void SegmentPlane::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if ( (cloud->width * cloud->height) == 0)
      return;

    sensor_msgs::PointCloud2Ptr cloudInProcFrame;

    // Transform the point cloud to the frame specified if any
    if ( !_processingFrame.empty() )
    {
      cloudInProcFrame.reset(new sensor_msgs::PointCloud2);
      ROS_DEBUG_STREAM("Transforming point cloud from frame " << cloud->header.frame_id << " to frame " << _processingFrame);
      pcl_ros::transformPointCloud(_processingFrame, *cloud, *cloudInProcFrame, _tfListener);
      cloudInProcFrame->header.frame_id = _processingFrame;
    }
    else
      *cloudInProcFrame = *cloud;

    //Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloudInProcFrame, *pclCloud);

    // Apply passthrough filter if required
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( !_axis.empty() )
    {
      pal::passThrough<pcl::PointXYZRGB>(pclCloud,
                                         _axis,
                                         _min, _max,
                                         passThroughCloud);

      if ( passThroughCloud->empty() )
      {
        //if all points get removed after the pass-through filtering just publish empty point clouds
        //and stop processing
        publishEmptyClouds(pclCloud->header.stamp, pclCloud->header.frame_id);
        return;
      }
    }
    else
      passThroughCloud = pclCloud;

    // Downsample the point cloud if required
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclDownSampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( _downSamplingSize > 0 )
      pal::downSample<pcl::PointXYZRGB>(passThroughCloud, pclDownSampledCloud, _downSamplingSize);
    else
      pclDownSampledCloud = passThroughCloud;

    if ( pclDownSampledCloud->points.size() < 10 )
    {
      ROS_INFO_STREAM("Not locating a plane because there are only " <<
                      pclDownSampledCloud->points.size() << " points");
      publishEmptyClouds(pclCloud->header.stamp, pclCloud->header.frame_id);
      return;
    }

    // Remove main plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclNonPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
    pal::planeSegmentation<pcl::PointXYZRGB>(pclDownSampledCloud,
                                             &pclPlaneCloud,
                                             &pclNonPlaneCloud,
                                             &planeCoeff);


    //filter outliers in the plane cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pclPlaneCloud->empty() )
      pclFilteredPlaneCloud = pclPlaneCloud;
    else
      pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclPlaneCloud, 25, 1.0,pclFilteredPlaneCloud);

    //filter outliers in the cloud not belonging to the main plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredNonPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pclNonPlaneCloud->empty() )
      pclFilteredNonPlaneCloud = pclNonPlaneCloud;
    else
      pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclNonPlaneCloud, 50, 0.1, pclFilteredNonPlaneCloud);

/*    ROS_INFO_STREAM("Processing:");
    ROS_INFO_STREAM("\tInput cloud:                 " << pclCloud->points.size() << " points");
    ROS_INFO_STREAM("\tAfter pass-through:          " << passThroughCloud->points.size() << " points");
    ROS_INFO_STREAM("\tAfter downsmapling:          " << pclDownSampledCloud->points.size() << " points");
    ROS_INFO_STREAM("\tPoints in plane:             " << pclPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tNon-plane points:            " << pclNonPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tOutliers in plane:           " << pclPlaneCloud->points.size() - pclFilteredPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tOutliers in non-plane:       " << pclNonPlaneCloud->points.size() - pclFilteredNonPlaneCloud->points.size() << " points");
*/
/*    for(int i=0; i<pclCloud->points.size(); i++){
      ROS_INFO_STREAM("\tInput cloud:                 " << pclCloud->points[i].x);
    }
*/
    publish(pclFilteredPlaneCloud,
            pclFilteredNonPlaneCloud,
            pclCloud->header.stamp,
            pclCloud->header.frame_id);


  }

//Separate nonplane point cloud into Pick and Place point clouds
  void SegmentPlane::nonplaneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& nonplaneCloud)
  {
    //Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*nonplaneCloud, *pclCloud);

    // Apply passthrough filter for Pick zone
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pickPassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( !_axisPick.empty() )
    {
      pal::passThrough<pcl::PointXYZRGB>(pclCloud,
                                         _axisPick,
                                         _minPick, _maxPick,
                                         pickPassThroughCloud);

      if ( pickPassThroughCloud->empty() )
      {
        //if all points get removed after the pass-through filtering just publish empty point clouds
        //and stop processing
        publishEmptyClouds(pclCloud->header.stamp, pclCloud->header.frame_id);
        return;
      }
    }
    else
      pickPassThroughCloud = pclCloud;
   
    //Apply passthrough filter for Place zone
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr placePassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( !_axisPlace.empty() )
    {
      pal::passThrough<pcl::PointXYZRGB>(pclCloud,
                                         _axisPlace,
                                         _minPlace, _maxPlace,
                                         placePassThroughCloud);

      if ( placePassThroughCloud->empty() )
      {
        //if all points get removed after the pass-through filtering just publish empty point clouds
        //and stop processing
        publishEmptyClouds(pclCloud->header.stamp, pclCloud->header.frame_id);
        return;
      }
    }
    else
      placePassThroughCloud = pclCloud;
   
    //Publish Pick zone pointcloud 
    if ( _pickCloudPub.getNumSubscribers() > 0 )
    {
      pickPassThroughCloud->header.stamp    = pclCloud->header.stamp;
      pickPassThroughCloud->header.frame_id = pclCloud->header.frame_id;
      _pickCloudPub.publish(pickPassThroughCloud);
    }
    //Publish Place zone pointcloud 
    if ( _placeCloudPub.getNumSubscribers() > 0 )
    {
      placePassThroughCloud->header.stamp    = pclCloud->header.stamp;
      placePassThroughCloud->header.frame_id = pclCloud->header.frame_id;
      _placeCloudPub.publish(placePassThroughCloud);
    }

  }


  void SegmentPlane::placeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& placeCloud)
  {
    ROS_INFO("HOLA");
    //Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*placeCloud, *pclCloud);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*pclCloud, minPt, maxPt);
    std::cout << "Min x: " << minPt.x << std::endl;
  }

//Detects corners from nonplane pointcloud with sum/diff of min/max points
  void SegmentPlane::pickCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pickCloud)
  {

    // To get the segmented cloud in PCL XYZ format (instead of XYZRGB)
    if ( (pickCloud->width * pickCloud->height) == 0)
      return;

    sensor_msgs::PointCloud2Ptr xyzcloudInProcFrame;

    // Transform the point cloud to the frame specified if any
    if ( !_processingFrame.empty() )
    {
      xyzcloudInProcFrame.reset(new sensor_msgs::PointCloud2);
      ROS_DEBUG_STREAM("Transforming point cloud from frame " << pickCloud->header.frame_id << " to frame " << _processingFrame);
      pcl_ros::transformPointCloud(_processingFrame, *pickCloud, *xyzcloudInProcFrame, _tfListener);
      xyzcloudInProcFrame->header.frame_id = _processingFrame;
    }
    else
      *xyzcloudInProcFrame = *pickCloud;

    //Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzpclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*xyzcloudInProcFrame, *xyzpclCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cacxyzpclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*xyzcloudInProcFrame, *cacxyzpclCloud);



//CONVEX HULL
    //It is necessary to extract plane coefficients to project the segmented towel point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cacpclPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cacpclNonPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
    pal::planeSegmentation<pcl::PointXYZRGB>(cacxyzpclCloud,
                                             &cacpclPlaneCloud,
                                             &cacpclNonPlaneCloud,
                                             &planeCoeff);
    // Project the model inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (xyzpclCloud);
    proj.setModelCoefficients (planeCoeff);
    proj.filter (*cloud_projected);
    //std::cerr << "PointCloud after projection has: " << cloud_projected->size () << " data points." << std::endl; //Debug

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> polygons;
    chull.setInputCloud (cloud_projected);
    //chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull, polygons);

    //std::cerr << "Concave hull has: " << cloud_hull->size () << " data points." << std::endl; //Debug

    // Publish the convex cloud
    if ( _convexCloudPub.getNumSubscribers() > 0 )
    {
      cloud_hull->header.stamp    = xyzpclCloud->header.stamp;
      cloud_hull->header.frame_id = xyzpclCloud->header.frame_id;
      _convexCloudPub.publish(cloud_hull);
    }


//Find MIN and MAX points:

//    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
//    pcl::copyPointCloud(cloud_xyz, pclFilteredNonPlaneCloud);

/*    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud_hull, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;*/

// Create markers to visualise corners in RVIZ
    visualization_msgs::Marker corner1_marker;
    visualization_msgs::Marker corner2_marker;
    visualization_msgs::Marker corner3_marker;
    visualization_msgs::Marker corner4_marker;
    visualization_msgs::MarkerArray corners_markers;
    visualization_msgs::Marker grasp_marker;

    //Initialize characteristics of the markers
    corner1_marker.header.frame_id = cloud_hull->header.frame_id;
    corner1_marker.id = 0;
    corner1_marker.type = visualization_msgs::Marker::SPHERE;
    corner1_marker.scale.x=0.01;
    corner1_marker.scale.y=0.01;
    corner1_marker.scale.z=0.01;
    corner1_marker.color.r = 0.0f;
    corner1_marker.color.g = 1.0f;
    corner1_marker.color.b = 1.0f;
    corner1_marker.color.a = 1.0;
    corner1_marker.lifetime = ros::Duration();

    //Same characteristics for all
    corner2_marker=corner1_marker;
    corner3_marker=corner1_marker;
    corner4_marker=corner1_marker;
    grasp_marker=corner1_marker;
    //Pink color for grasp point marker
    grasp_marker.color.r = 1.0f;
    grasp_marker.color.g = 0.0f;

// GET CORNERS
    //std::cout << "CONVEX SIZE: " << cloud_hull->points.size() << std::endl;
    pcl::PointXYZ pt_down_right, pt_down_left, pt_up_right, pt_up_left;
    float max_diff = cloud_hull->points[0].y - cloud_hull->points[0].z;
    float min_sum = cloud_hull->points[0].y + cloud_hull->points[0].z;
    float min_diff = max_diff;
    float max_sum = min_sum;
    //std::cout << "max diff: " << max_diff << "min sum: " << min_sum << std::endl;

    //Init
    pt_down_right.x=cloud_hull->points[0].x;
    pt_down_right.y=cloud_hull->points[0].y;
    pt_down_right.z=cloud_hull->points[0].z;
    pt_up_left.x=cloud_hull->points[0].x;
    pt_up_left.y=cloud_hull->points[0].y;
    pt_up_left.z=cloud_hull->points[0].z;
    pt_up_right.x=cloud_hull->points[0].x;
    pt_up_right.y=cloud_hull->points[0].y;
    pt_up_right.z=cloud_hull->points[0].z;
    pt_down_left.x=cloud_hull->points[0].x;
    pt_down_left.y=cloud_hull->points[0].y;
    pt_down_left.z=cloud_hull->points[0].z;

    //Get corners
    for(int i=0; i<cloud_hull->points.size(); i++)
    {
      float diff = cloud_hull->points[i].y - cloud_hull->points[i].z;
      float sum = cloud_hull->points[i].y + cloud_hull->points[i].z;
      if(min_sum > sum)
      {
          min_sum=sum;
          pt_up_left.x=cloud_hull->points[i].x;
          pt_up_left.y=cloud_hull->points[i].y;
          pt_up_left.z=cloud_hull->points[i].z;
      }
      if(max_sum < sum)
      {
          max_sum=sum;
          pt_down_right.x=cloud_hull->points[i].x;
          pt_down_right.y=cloud_hull->points[i].y;
          pt_down_right.z=cloud_hull->points[i].z;
      }
      if(min_diff > diff)
      {
          min_diff=diff;
          pt_down_left.x=cloud_hull->points[i].x;
          pt_down_left.y=cloud_hull->points[i].y;
          pt_down_left.z=cloud_hull->points[i].z;
      }
      if(max_diff < diff)
      {
          max_diff=diff;
          pt_up_right.x=cloud_hull->points[i].x;
          pt_up_right.y=cloud_hull->points[i].y;
          pt_up_right.z=cloud_hull->points[i].z;
      }
    }

//Real time detected points
      //Debug
      /*std::cout << "\033[1;36m PT DOWN LEFT: " << pt_down_left.z << ", " << pt_down_left.y << ", " << pt_down_left.x << std::endl;
      std::cout << "\033[1;36m PT UP LEFT: " << pt_up_left.z << ", " << pt_up_left.y << ", " << pt_up_left.x << std::endl;
      std::cout << "\033[1;36m PT DOWN RIGHT: " << pt_down_right.z << ", " << pt_down_right.y << ", " << pt_down_right.x << std::endl;
      std::cout << "\033[1;36m PT UP RIGHT: " << pt_up_right.z << ", " << pt_up_right.y << ", " << pt_up_right.x << std::endl; */

      corner1_marker.id = 1;
      corner1_marker.pose.position.x=pt_down_right.x;
      corner1_marker.pose.position.y=pt_down_right.y;
      corner1_marker.pose.position.z=pt_down_right.z;
      corner2_marker.id = 2;
      corner2_marker.pose.position.x=pt_up_left.x;
      corner2_marker.pose.position.y=pt_up_left.y;
      corner2_marker.pose.position.z=pt_up_left.z;
      corner3_marker.id = 3;
      corner3_marker.pose.position.x=pt_up_right.x;
      corner3_marker.pose.position.y=pt_up_right.y;
      corner3_marker.pose.position.z=pt_up_right.z;
      corner4_marker.id = 4;
      corner4_marker.pose.position.x=pt_down_left.x;
      corner4_marker.pose.position.y=pt_down_left.y;
      corner4_marker.pose.position.z=pt_down_left.z;

      //Add corners to the array to publish them
      corners_markers.markers.push_back(corner1_marker);
      corners_markers.markers.push_back(corner2_marker);
      corners_markers.markers.push_back(corner3_marker);
      corners_markers.markers.push_back(corner4_marker);

      //Publish corners
      if ( _cornersMarkersPub.getNumSubscribers() > 1 )
      {
        _cornersMarkersPub.publish(corners_markers);
	corners_markers.markers.clear();
      }



//Robust corner points - sliding window

/*    std::cout << "\043[1;36m PT UP RIGHT: " << pt_up_right.z << ", " << pt_up_right.y << ", " << pt_up_right.x << std::endl;
    current_sum_c1.x += pt_down_right.x;
    current_sum_c1.y += pt_down_right.y;
    current_sum_c1.z += pt_down_right.z;
    current_sum_c2.x += pt_up_left.x;
    current_sum_c2.y += pt_up_left.y;
    current_sum_c2.z += pt_up_left.z;
    current_sum_c3.x += pt_up_right.x;
    current_sum_c3.y += pt_up_right.y;
    current_sum_c3.z += pt_up_right.z;
    current_sum_c4.x += pt_down_left.x;
    current_sum_c4.y += pt_down_left.y;
    current_sum_c4.z += pt_down_left.z;

    n_frames+=1;
    std::cout << "n frames ------> " << n_frames << std::endl;
    if(n_frames>2)
    {
      std::cout << "\033[1;36m PT DOWN LEFT: " << current_sum_c4.z/n_frames << ", " << current_sum_c4.y/n_frames << ", " << current_sum_c4.x/n_frames << std::endl;
      std::cout << "\033[1;36m PT UP LEFT: " << current_sum_c2.z/n_frames << ", " << current_sum_c2.y/n_frames << ", " << current_sum_c2.x/n_frames << std::endl;
      std::cout << "\033[1;36m PT DOWN RIGHT: " << current_sum_c1.z/n_frames << ", " << current_sum_c1.y/n_frames << ", " << current_sum_c1.x/n_frames << std::endl;
      std::cout << "\033[1;36m PT UP RIGHT: " << current_sum_c3.z/n_frames << ", " << current_sum_c3.y/n_frames << ", " << current_sum_c3.x/n_frames << std::endl;

      corner1_marker.id = 1;
      corner1_marker.pose.position.x=current_sum_c1.x/n_frames;
      corner1_marker.pose.position.y=current_sum_c1.y/n_frames;
      corner1_marker.pose.position.z=current_sum_c1.z/n_frames;
      corner2_marker.id = 2;
      corner2_marker.pose.position.x=current_sum_c2.x/n_frames;
      corner2_marker.pose.position.y=current_sum_c2.y/n_frames;
      corner2_marker.pose.position.z=current_sum_c2.z/n_frames;
      corner3_marker.id = 3;
      corner3_marker.pose.position.x=current_sum_c3.x/n_frames;
      corner3_marker.pose.position.y=current_sum_c3.y/n_frames;
      corner3_marker.pose.position.z=current_sum_c3.z/n_frames;
      corner4_marker.id = 4;
      corner4_marker.pose.position.x=current_sum_c4.x/n_frames;
      corner4_marker.pose.position.y=current_sum_c4.y/n_frames;
      corner4_marker.pose.position.z=current_sum_c4.z/n_frames;

      //Add corners to the array to publish them
      corners_markers.markers.push_back(corner1_marker);
      corners_markers.markers.push_back(corner2_marker);
      corners_markers.markers.push_back(corner3_marker);
      corners_markers.markers.push_back(corner4_marker);

      //Publish corners
      _cornersMarkersPub.publish(corners_markers);
      corners_markers.markers.clear();

      //Reset for next window
      n_frames=0;
      current_sum_c1.x = 0;
      current_sum_c1.y = 0;
      current_sum_c1.z = 0;
      current_sum_c2.x = 0;
      current_sum_c2.y = 0;
      current_sum_c2.z = 0;
      current_sum_c3.x = 0;
      current_sum_c3.y = 0;
      current_sum_c3.z = 0;
      current_sum_c4.x = 0;
      current_sum_c4.y = 0;
      current_sum_c4.z = 0;

      //Initialize corners for grasp point selection
      pt_down_right.x = current_sum_c1.x/n_frames;
      pt_down_right.y = current_sum_c1.y/n_frames;
      pt_down_right.z = current_sum_c1.z/n_frames;
      pt_up_left.x = current_sum_c2.x/n_frames;
      pt_up_left.y = current_sum_c2.y/n_frames;
      pt_up_left.z = current_sum_c2.z/n_frames;
      pt_up_right.x = current_sum_c3.x/n_frames;
      pt_up_right.y = current_sum_c3.y/n_frames;
      pt_up_right.z = current_sum_c3.z/n_frames;
      pt_down_left.x = current_sum_c4.x/n_frames;
      pt_down_left.y = current_sum_c4.y/n_frames;
      pt_down_left.z = current_sum_c4.z/n_frames;

    }*/

//EDGES and GRASP POINT (Should take the closest edge to the robot (TF robot))
/*        // Compute edges size
        pcl::PointXYZ grasp_point;
        float edge1 = abs(pt_down_left.y - pt_down_right.y);
        float edge2 = abs(pt_up_left.z - pt_down_left.z);
        edge2 = sqrt(pow(abs(pt_up_left.z-pt_down_left.z),2)+pow(abs(pt_up_left.y-pt_down_left.y),2));
        edge1 = sqrt(pow(abs(pt_down_right.z-pt_down_left.z),2)+pow(abs(pt_down_right.y-pt_down_left.y),2));
        //std::cout << "Edges ahora: " << edge1 << ", " << edge2 << std::endl; //Debug
        float mid_pt;
        double u1, u2; //Edge direction vector components
        std_msgs::Float64 grasp_angle;
        float cos_alpha, alpha;
        std_msgs::Float64 garment_edge;

        // Check if the object's shape is square or a rectangle
        if(abs(edge1-edge2)>0.05)
        {
          // Get mid point of longest edge
          if(edge1 > edge2)
          {
            //std::cout << "Edge1!" << std::endl; //Debug
            grasp_point.z = pt_down_left.z + (pt_down_right.z - pt_down_left.z)/2;
            grasp_point.y = pt_down_left.y - (abs(pt_down_left.y - pt_down_right.y)/2); //REVISAR?
            grasp_point.x = pt_down_left.x-0.02;
            garment_edge.data = abs(edge2);
            u1 = pt_down_left.z - pt_down_right.z;    //X direction of edge vector
            u2 = pt_down_left.y - pt_down_right.y;    //Y direction of edge vector
            cos_alpha = (abs(u2))/(sqrt(pow(u1,2)+pow(u2,2)));
            alpha = acos(cos_alpha);
            //std::cout << "U!: " << u1 << " / " << u2 << std::endl; //Debug
            if(0.2 > alpha > 0)
            {
              grasp_angle.data = 0;
              //std::cout << "HORIZONTAL: E1 > 0-0.4" << std::endl; //Debug
            }
            else //alpha > 0.2
            {
              if(u1>0) //pt_down_left.z > pt_down_right.z = Diagonal izq
              {
                grasp_angle.data = 1;
                //std::cout << "DIAGONAL IZQ: E1>0.2 y U1>0" << std::endl; //Debug
              }
              else // Diagonal der
              {  //REVISAR!!!!
                //std::cout << "DIAGONAL DER: E1>0.2 y U1<0" << std::endl; //Debug
                grasp_angle.data = 2;
              }
            }
          }
          else
          {
            //std::cout << "Edge2!" << std::endl; //Debug
            grasp_point.z = pt_down_left.z + (abs(pt_down_left.z - pt_up_left.z)/2); //REVISAR!
            grasp_point.y = pt_down_left.y + (pt_up_left.y - pt_down_left.y)/2; //REVISAR?
            grasp_point.x = pt_down_left.x-0.02;
            garment_edge.data = abs(edge1);
            u1 = pt_up_left.z - pt_down_left.z;    //X direction of edge vector
            u2 = pt_up_left.y - pt_down_left.y;    //Y direction of edge vector
            cos_alpha = (abs(u2))/(sqrt(pow(u1,2)+pow(u2,2)));
            alpha = acos(cos_alpha);
            if(alpha > 1)
            {
              grasp_point.z = pt_down_right.z + (abs(pt_down_right.z - pt_up_right.z)/2); //REVISAR!
              grasp_point.y = pt_down_right.y + (pt_up_right.y - pt_down_right.y)/2; //REVISAR?
              grasp_point.x = pt_down_right.x-0.02;
              grasp_angle.data = 3;
              //std::cout << "VERTICAL: E2 > 1" << std::endl; //Debug
            }
            if(1 > alpha > 0.3)
            {
              grasp_point.z = pt_down_right.z + (abs(pt_down_right.z - pt_up_right.z)/2); //REVISAR!
              grasp_point.y = pt_down_right.y + (pt_up_right.y - pt_down_right.y)/2; //REVISAR?
              grasp_point.x = pt_down_right.x-0.02;
              //std::cout << "DIAGONAL DER: E2 > 0.3-1.2" << std::endl; //Debug
              grasp_angle.data = 2;
            }

          }
        }
        else //Squared object -> Get mid point of closet edge
        {
          //std::cout << "Square!" << std::endl; //Debug
          grasp_point.z = pt_down_left.z + (pt_down_right.z - pt_down_left.z)/2;
          grasp_point.y = pt_down_left.y - (abs(pt_down_left.y - pt_down_right.y)/2);
          grasp_point.x = pt_down_left.x-0.02;
          u1 = pt_down_left.z - pt_down_right.z;    //X direction of edge vector
          u2 = pt_down_left.y - pt_down_right.y;    //Y direction of edge vector
          garment_edge.data = abs(edge2);
        }

        //std::cout << "\033[1;36m GRASP POINT: " << grasp_point.z << ", " << grasp_point.y << ", " << grasp_point.x << std::endl; //Debug
        grasp_marker.pose.position.x=grasp_point.x;
        grasp_marker.pose.position.y=grasp_point.y;
        grasp_marker.pose.position.z=grasp_point.z;

        if ( _graspPointPub.getNumSubscribers() > 1 )
          _graspPointPub.publish(grasp_marker);
        _garmentEdgePub.publish(garment_edge);


// Get angle edge
        //Tendre que cambiarlo para cada edge!!
        //double u1 = pt_up_left.z - pt_down_left.z;    //X direction of edge vector
        //double u2 = pt_up_left.y - pt_down_left.y;    //Y direction of edge vector
        //double cos_alpha = (abs(pow(pt_down_left.z,2)))/(sqrt(pow(pt_down_left.z,2)+pow(pt_down_left.y))*sqrt(pow(pt_down_left.z,2)));
        double hola1 = abs(u2);
        double hola2 = sqrt(pow(u1,2)+pow(u2,2));
        cos_alpha = (abs(u2))/(sqrt(pow(u1,2)+pow(u2,2)));
        alpha = acos(cos_alpha);
        //std::cout << "DOWN LEFT X: " << pt_down_left.z << "  -UP LEFT X: " << pt_up_left.z << std::endl;
        //std::cout << "DOWN LEFT Y: " << pt_down_left.y << "  -UP LEFT Y: " << pt_up_left.y << std::endl;
        //std::cout << "U1: " << u1 << "   U2: " << u2 << std::endl;
        //std::cout << "HOLA: " << hola1 << "   HOLA2: " << hola2 << std::endl;
        //std::cout << "COS ALPHA: " << cos_alpha << std::endl;
        //std::cout << "ALPHA: " << alpha << std::endl; 

//        std_msgs::Float64 grasp_angle;
//        grasp_angle.data = alpha;
        _graspPointAnglePub.publish(grasp_angle);
*/
//    }

  }

  void SegmentPlane::publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& planeCloud,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nonPlaneCloud,
                                 pcl::uint64_t& stamp,
                                 const std::string& frameId)
  {
    if ( _planeCloudPub.getNumSubscribers() > 0 )
    {
      planeCloud->header.stamp    = stamp;
      planeCloud->header.frame_id = frameId;
      _planeCloudPub.publish(planeCloud);
    }

    if ( _nonPlaneCloudPub.getNumSubscribers() > 0 )
    {
      nonPlaneCloud->header.stamp    = stamp;
      nonPlaneCloud->header.frame_id = frameId;
      _nonPlaneCloudPub.publish(nonPlaneCloud);
    }
  }

  void SegmentPlane::start()
  {
    _cloudSub = _nh.subscribe("cloud", 1, &SegmentPlane::cloudCallback, this);
    //_segmCloudSub = _nh.subscribe("segmcloud2", 1, &SegmentPlane::segmCloudCallback, this);
    _pickCloudSub = _nh.subscribe("pickcloud", 1, &SegmentPlane::pickCloudCallback, this);
    _placeCloudSub = _nh.subscribe("placecloud", 1, &SegmentPlane::placeCloudCallback, this);
    _nonplaneCloudSub = _nh.subscribe("segmcloud", 1, &SegmentPlane::nonplaneCloudCallback, this);
    _enabled = true;
  }

  void SegmentPlane::stop()
  {
    _cloudSub.shutdown();
    //_segmCloudSub.shutdown();
    _pickCloudSub.shutdown();
    _enabled = false;
  }

  void SegmentPlane::run()
  {
    ros::Rate loopRate(_rate);

    double halfPeriod = 0.5*1.0/_rate;

    while ( ros::ok() )
    {
      bool anySubscriber = _planeCloudPub.getNumSubscribers() > 0 ||
                           _nonPlaneCloudPub.getNumSubscribers() > 0;


      if ( !_enabled && anySubscriber )
      {
        ROS_INFO("Enabling node because there are subscribers");
        start();
      }
      else if ( _enabled && !anySubscriber )
      {
        ROS_INFO("Disabling node because there are no subscribers");
        stop();
      }

      //check for subscriber's callbacks
      _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

      loopRate.sleep();
    }
  }

} //pal


int main(int argc, char**argv)
{
  ros::init (argc, argv, "segment_table");

  ros::NodeHandle nh, pnh("~");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  pal::SegmentPlane locator(nh, pnh);

  locator.run();

  return 0;
}
