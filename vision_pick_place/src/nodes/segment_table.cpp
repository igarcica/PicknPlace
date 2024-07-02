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
#include <vision_pick_place/planesegmConfig.h>

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

#include <dynamic_reconfigure/server.h>

// Std C++ headers
#include <string>

namespace pal {

  class SegmentPlane {
  public:
    SegmentPlane(ros::NodeHandle& nh,
                 ros::NodeHandle& pnh);

    virtual ~SegmentPlane();

    void run();

    double _plane_outliers_min_k;
    double _plane_outliers_stddev_mult;
    double _nonplane_outliers_min_k;
    double _nonplane_outliers_stddev_mult;

  protected:

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    //void segmCloudCallback(const sensor_msgs::PointCloud2ConstPtr& segmCloud);
    void pickCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pickCloud);
    void placeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& placeCloud);
    void nonplaneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& nonplaneCloud);
    //void garmentCloudCallback(const sensor_msgs::PointCloud2ConstPtr& garmentCloud);
    visualization_msgs::MarkerArray computeCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud, std::string axis, double min, double max);
  

    void start();
    void stop();

    void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& planeCloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nonPlaneCloud,
                 pcl::uint64_t& stamp,
                 const std::string& frameId);

    void publishEmptyClouds(pcl::uint64_t& stamp,
                            const std::string& frameId);

    void normalizeVector(std::vector<double>& v);

    void dynam_CB(const vision_pick_place::planesegmConfig &config, uint32_t level);

    dynamic_reconfigure::Server<vision_pick_place::planesegmConfig> reconf_server;
    dynamic_reconfigure::Server<vision_pick_place::planesegmConfig>::CallbackType reconf_cb;

    ros::NodeHandle& _nh, _pnh;
    ros::CallbackQueue _cbQueue;
    bool _enabled;
    double _rate;

    tf::TransformListener _tfListener;

    // frame in which the point cloud will be transformed
    std::string _processingFrame;

    // pass-through filter parameters:
    std::string _axis, _axisPick, _z_axisPick, _axisPlace, _axisRobotarm, _axisGridX, _dataType;
    double _min, _max, _minPick, _maxPick, _z_minPick, _z_maxPick, _minPlace, _maxPlace, _minRobotarm, _maxRobotarm, _minGridX, _maxGridX;

    // downsampling filter parameters:
    double _downSamplingSize;

    // ROS subscribers
    ros::Subscriber _cloudSub;
    //ros::Subscriber _segmCloudSub;
    ros::Subscriber _pickCloudSub;
    ros::Subscriber _placeCloudSub;
    ros::Subscriber _nonplaneCloudSub;

    ros::Subscriber _garmentCloudSub;

    // ROS publishers
    ros::Publisher _planeCloudPub;
    ros::Publisher _nonPlaneCloudPub;
    ros::Publisher _mainPlanePosePub;
    ros::Publisher _pickCloudPub;
    ros::Publisher _placeCloudPub;

    ros::Publisher _garmentCloudPub;
    /*ros::Publisher _grid1CloudPub;
    ros::Publisher _grid2CloudPub;
    ros::Publisher _grid3CloudPub;
    ros::Publisher _grid4CloudPub;*/

    ros::Publisher _convexCloudPub;
    ros::Publisher _cornersMarkersPub;
    ros::Publisher _placeCornersMarkersPub;
    //ros::Publisher _metricMarkersPub;
    ros::Publisher _graspPointPub;
    ros::Publisher _garmentEdgePub;
    ros::Publisher _pileHeightPub;

    //Variables globales:
    int n_frames;
    pcl::PointXYZ current_sum_c1;
    pcl::PointXYZ current_sum_c2;
    pcl::PointXYZ current_sum_c3;
    pcl::PointXYZ current_sum_c4;
    ros::Publisher _graspPointAnglePub;

    double x_thr_canon, y_thr_canon, min_z_canon, grid1_size, grid2_size, grid3_size, grid4_size;

  };

  SegmentPlane::SegmentPlane(ros::NodeHandle& nh,
                                     ros::NodeHandle& pnh):
    _nh(nh),
    _pnh(pnh),
    _enabled(false),
    _rate(1),
    _plane_outliers_min_k(25),
    _plane_outliers_stddev_mult(1.0),
    _nonplane_outliers_min_k(50),
    _nonplane_outliers_stddev_mult(0.1),
    _processingFrame(""),
    _axis(""),
    _min(0.5),
    _max(10),
    _axisPick(""),
    _minPick(0.5),
    _maxPick(10),
    _z_axisPick(""),
    _z_minPick(10),
    _z_maxPick(10),
    _axisPlace(""),
    _minPlace(0.5),
    _maxPlace(10),
    _axisRobotarm(""),
    _minRobotarm(0.5),
    _maxRobotarm(10),
    _axisGridX(""),
    _minGridX(0.5),
    _maxGridX(10),
    _dataType(""),
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
    pnh.param<std::string>("passthrough_z_axis_pick", _z_axisPick, _z_axisPick);
    pnh.param<double>("passthrough_z_min_pick", _z_minPick, _z_minPick);
    pnh.param<double>("passthrough_z_max_pick", _z_maxPick, _z_maxPick);
    pnh.param<std::string>("passthrough_axis_place", _axisPlace, _axisPlace);
    pnh.param<double>("passthrough_min_place", _minPlace, _minPlace);
    pnh.param<double>("passthrough_max_place", _maxPlace, _maxPlace);
    pnh.param<std::string>("passthrough_axis_robotarm", _axisRobotarm, _axisRobotarm);
    pnh.param<double>("passthrough_min_robotarm", _minRobotarm, _minRobotarm);
    pnh.param<double>("passthrough_max_robotarm", _maxRobotarm, _maxRobotarm);
    pnh.param<std::string>("passthrough_axis_gridx", _axisGridX, _axisGridX);
    pnh.param<double>("passthrough_min_gridX", _minGridX, _minGridX);
    pnh.param<double>("passthrough_max_gridX", _maxGridX, _maxGridX);
    pnh.param<std::string>("data_type", _dataType, _dataType);
    pnh.param<double>("downsampling_size", _downSamplingSize, _downSamplingSize);

    ROS_INFO_STREAM("The node will operate at maximum " << _rate << " Hz");

    reconf_cb = boost::bind(&SegmentPlane::dynam_CB, this, _1, _2);
    reconf_server.setCallback(reconf_cb);

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

    if(_dataType == "gr_z")
    {
      ROS_INFO("Getting Grasp Z parameters");
      _minRobotarm = -0.2;
      x_thr_canon = 0.037;
      y_thr_canon = -0.077;
      min_z_canon = 0.41;
      grid1_size = 186;
      grid2_size = 185;
      grid3_size = 146;
      grid4_size = 153;
    }
    else if(_dataType == "pl_z")
    {
      ROS_INFO("Getting Placing Z parameters");
      _minRobotarm = -1.0;
      x_thr_canon = 0.015;
      y_thr_canon = -0.29;
      min_z_canon = 0.71;
      grid1_size = 290;
      grid2_size = 267;
      grid3_size = 256;
      grid4_size = 237;
    }
    else if(_dataType == "plg_z")
    {
      ROS_INFO("Placing grasp Z parameters");
      _minRobotarm = -0.38;
      x_thr_canon = 0.019;
      y_thr_canon = -0.23;
      min_z_canon = 0.65;
      grid1_size = 171;
      grid2_size = 144;
      grid3_size = 197;
      grid4_size = 158;
    }
    else if(_dataType == "o2_gr_z")
    {
      ROS_INFO("Getting Grasp Z parameters");
      _minRobotarm = -0.2;
      x_thr_canon = 0.033;
      y_thr_canon = -0.083;
      min_z_canon = 0.42;
      grid1_size = 204;
      grid2_size = 181;
      grid3_size = 215;
      grid4_size = 213;
    }
    else if(_dataType == "o2_pl_z")
    {
      ROS_INFO("Getting Placing Z parameters");
      _minRobotarm = -1.0;
      x_thr_canon = 0.015;
      y_thr_canon = -0.3;
      min_z_canon = 0.75;
      grid1_size = 210;
      grid2_size = 220;
      grid3_size = 217;
      grid4_size = 206;
    }
    else if(_dataType == "o2_plg_z")
    {
      ROS_INFO("Placing grasp Z parameters");
      _minRobotarm = -1;
      x_thr_canon = 0.012;
      y_thr_canon = -0.24;
      min_z_canon = 0.66;
      grid1_size = 184;
      grid2_size = 206;
      grid3_size = 223;
      grid4_size = 223;
    }
    else
      ROS_INFO("Dividing grid");

    _planeCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane", 1);
    _nonPlaneCloudPub = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("nonplane", 1);
    _mainPlanePosePub = _pnh.advertise<geometry_msgs::PoseStamped>("plane_pose", 1);
    _pickCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("pick", 1);
    _placeCloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("place", 1);

    _garmentCloudPub = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("garment", 1);
    /*_grid1CloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("grid1", 1);
    _grid2CloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("grid2", 1);
    _grid3CloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("grid3", 1);
    _grid4CloudPub    = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("grid4", 1);*/

    _convexCloudPub   = _pnh.advertise< pcl::PointCloud<pcl::PointXYZ> >("convex", 1);
    _cornersMarkersPub = _pnh.advertise<visualization_msgs::MarkerArray>("pick_corners", 1);
    _placeCornersMarkersPub = _pnh.advertise<visualization_msgs::MarkerArray>("place_corners", 1);
    //_metricMarkersPub = _pnh.advertise<visualization_msgs::MarkerArray>("metric", 1);
    _graspPointPub    = _pnh.advertise<visualization_msgs::Marker>("grasp_point", 1);

    n_frames=0;
    _graspPointAnglePub    = _pnh.advertise<std_msgs::Float64>("grasp_angle", 1);
    _garmentEdgePub   = _pnh.advertise<std_msgs::Float64>("garment_edge", 1);
    _pileHeightPub    = _pnh.advertise<std_msgs::Float64>("pile_height", 1);

  }

  SegmentPlane::~SegmentPlane()
  {
  }

  void SegmentPlane::dynam_CB(const vision_pick_place::planesegmConfig &config, uint32_t level)
  {
    std::cout << "SegmentPlane: Reconfigure parameters updated" << std::endl;
    _plane_outliers_min_k = config.plane_outliers_min_k;
    _plane_outliers_stddev_mult = config.plane_outliers_stddev_mult;
    _nonplane_outliers_min_k = config.nonplane_outliers_min_k;
    _nonplane_outliers_stddev_mult = config.nonplane_outliers_stddev_mult;
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
      ROS_INFO_STREAM("Transforming point cloud from frame " << cloud->header.frame_id << " to frame " << _processingFrame);
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
      pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclPlaneCloud, _plane_outliers_min_k, _plane_outliers_stddev_mult, pclFilteredPlaneCloud);

    //filter outliers in the cloud not belonging to the main plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredNonPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pclNonPlaneCloud->empty() )
      pclFilteredNonPlaneCloud = pclNonPlaneCloud;
    else
      pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclNonPlaneCloud, _nonplane_outliers_min_k, _nonplane_outliers_stddev_mult, pclFilteredNonPlaneCloud);

    /*ROS_INFO_STREAM("Processing:");
    ROS_INFO_STREAM("\tInput cloud:                 " << pclCloud->points.size() << " points");
    ROS_INFO_STREAM("\tAfter pass-through:          " << passThroughCloud->points.size() << " points");
    ROS_INFO_STREAM("\tAfter downsmapling:          " << pclDownSampledCloud->points.size() << " points");
    ROS_INFO_STREAM("\tPoints in plane:             " << pclPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tNon-plane points:            " << pclNonPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tOutliers in plane:           " << pclPlaneCloud->points.size() - pclFilteredPlaneCloud->points.size() << " points");
    ROS_INFO_STREAM("\tOutliers in non-plane:       " << pclNonPlaneCloud->points.size() - pclFilteredNonPlaneCloud->points.size() << " points");
    */
    /*for(int i=0; i<pclCloud->points.size(); i++){
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
    ROS_DEBUG("SegmentPlace: nonplaneCloud Callback");

    if ( (nonplaneCloud->width * nonplaneCloud->height) == 0)
    {
      ROS_DEBUG("SegmentPlace: Empty nonplaneCloud");
      return;
    }

    //Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*nonplaneCloud, *pclCloud);

    // Apply passthrough filter for Pick zone
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pickPassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pickPassThroughCloud = filterPointCloud(pclCloud, _axisPick, _minPick, _maxPick);
    pickPassThroughCloud = filterPointCloud(pickPassThroughCloud, _z_axisPick, _z_minPick, _z_maxPick);

    //Apply passthrough filter for Place zone
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr placePassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    placePassThroughCloud = filterPointCloud(pclCloud, _axisPlace, _minPlace, _maxPlace);

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

    /////// COMPUTE DEPTH MEANS (DEFORMATION METRIC)

    //Transform cloud to PCL format
    //XYZ or XYZRGB? minPt can be used with XYZRGB?

    //Publish Garment without gripper pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr garmentPassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    garmentPassThroughCloud = filterPointCloud(pclCloud, _axisRobotarm, _minRobotarm, _maxRobotarm);

    //Publish garment (without robot arm) pointcloud
    if ( _garmentCloudPub.getNumSubscribers() > 0 )
    {
      garmentPassThroughCloud->header.stamp    = pclCloud->header.stamp;
      garmentPassThroughCloud->header.frame_id = pclCloud->header.frame_id;
      _garmentCloudPub.publish(garmentPassThroughCloud);
    }
  }


  //Subscribe to garment (without robot arm) and compute depth
  /*void SegmentPlane::garmentCloudCallback(const sensor_msgs::PointCloud2ConstPtr& garmentCloud)
  {
    ROS_DEBUG("SegmentPlace: garmentCloud Callback");

    if ( (garmentCloud->width * garmentCloud->height) == 0)
    {
      ROS_DEBUG("SegmentPlace: Empty garmentCloud");
      return;
    }

    //Transform cloud to PCL format
    //For extracting min and max points
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclDepthCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*garmentCloud, *pclDepthCloud);

    //For filtering and getting the grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclDepthRGBCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*garmentCloud, *pclDepthRGBCloud);

    // pcl::copyPointCloud();

    //Divide garment's point cloud in a grid (of 'param' squares)
    //Get min and max points of the garment point cloud for division
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*pclDepthCloud, minPt, maxPt);
    //pcl::getMinMax3D (*garmentPassThroughCloud, minPt, maxPt);
    double x_threshold = minPt.x + (maxPt.x - minPt.x)/2; //2 will be defined
    double y_threshold = minPt.y + (maxPt.y - minPt.y)/2; //2 will be defined
    double z_threshold = minPt.z + (maxPt.z - minPt.z)/2;
    // std::cout << "X threshold: " << x_threshold << " / Y threshold: " << y_threshold << " / Z threshold: " << z_threshold << std::endl;
    // std::cout << "Min X: " << minPt.x << " / Min Y: " << minPt.y << " / Min Z: " << minPt.z << std::endl;
    // std::cout << "Max X: " << maxPt.x << " / Max Y: " << maxPt.y << " / Max Z: " << maxPt.z << std::endl;
    

    if (_dataType.empty())
    {
      //ROS_INFO("Getting custom parameters");
      x_thr_canon = x_threshold;
      y_thr_canon = y_threshold;
      min_z_canon = minPt.z;
      grid1_size = 1;
      grid2_size = 1;
      grid3_size = 1;
      grid4_size = 1;
    }
    //min_z_canon = minPt.z;
    //CANONICAL VALUES
    //std::cout << "CANONICAL Values: " << "Grid sizes: " << grid1_size << " / " << grid2_size << " / " << grid3_size << " / " << grid4_size << std::endl;

    //Apply passthrough filters for division - How to make it configurable?
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr izqPassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr derPassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid1PassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid2PassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid3PassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid4PassThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( !_axisGridX.empty() )
    {
      //std::cout << "Dividing grid..." << std::endl;
      pal::passThrough<pcl::PointXYZRGB>(pclDepthRGBCloud, //Mitad izquierda
                                         "x",
                                         minPt.x, x_thr_canon, //Min defined before (minPt.x?)
                                         //minPt.x, x_threshold,
                                         //minPt.x, 0.05,
                                         //minPt.x, 0.032,
                                         izqPassThroughCloud);
      pal::passThrough<pcl::PointXYZRGB>(izqPassThroughCloud, //Cuadrante izquierda superior
                                         "y",
                                         minPt.y, y_thr_canon,
                                         //minPt.y, y_threshold,
                                         //minPt.y, -0.07,
                                         //minPt.y, -0.25,
                                         grid4PassThroughCloud);
      pal::passThrough<pcl::PointXYZRGB>(izqPassThroughCloud, //Cuadrante izquierda inferior
                                         "y",
                                         y_thr_canon, maxPt.y,
                                         //y_threshold, maxPt.y,
                                         //-0.07, maxPt.y,
                                         //-0.25, maxPt.y,
                                         grid2PassThroughCloud);

      pal::passThrough<pcl::PointXYZRGB>(pclDepthRGBCloud, //Mitad derecha
                                         _axisGridX, //y axis
                                         x_thr_canon, _maxGridX, //max defined before
                                         //x_threshold, _maxGridX,
                                         //0.05, _maxGridX,
                                         //0.032, _maxGridX,
                                         derPassThroughCloud);
      pal::passThrough<pcl::PointXYZRGB>(derPassThroughCloud, //Cuadrante derecha superior
                                         "y",
                                         minPt.y, y_thr_canon,
                                         //minPt.y, y_threshold,
                                         //minPt.y, -0.07,
                                         //minPt.y, -0.25,
                                         grid3PassThroughCloud);
      pal::passThrough<pcl::PointXYZRGB>(derPassThroughCloud, //Cuadrante derecha inferior
                                         "y",
                                         y_thr_canon, maxPt.y,
                                         //y_threshold, maxPt.y,
                                         //-0.07, maxPt.y,
                                         //-0.25, maxPt.y,
                                         grid1PassThroughCloud);
    }
    else{
      grid1PassThroughCloud = pclDepthRGBCloud;
      grid2PassThroughCloud = pclDepthRGBCloud;
      grid3PassThroughCloud = pclDepthRGBCloud;
      grid4PassThroughCloud = pclDepthRGBCloud;
    }

    // std::cout << "TOTAL Size: " << pclDepthRGBCloud->size() << std::endl;
    // std::cout << "Size Grid1: " << grid1PassThroughCloud->size() << std::endl;
    // std::cout << "Size Grid2: " << grid2PassThroughCloud->size() << std::endl;
    // std::cout << "Size Grid3: " << grid3PassThroughCloud->size() << std::endl;
    // std::cout << "Size Grid4: " << grid4PassThroughCloud->size() << std::endl;
    
    //Publish grids pointclouds
    if ( _grid1CloudPub.getNumSubscribers() > 0 )
    {
      grid1PassThroughCloud->header.stamp    = pclDepthRGBCloud->header.stamp;
      grid1PassThroughCloud->header.frame_id = pclDepthRGBCloud->header.frame_id;
      _grid1CloudPub.publish(grid1PassThroughCloud);
    }
    if ( _grid2CloudPub.getNumSubscribers() > 0 )
    {
      grid2PassThroughCloud->header.stamp    = pclDepthRGBCloud->header.stamp;
      grid2PassThroughCloud->header.frame_id = pclDepthRGBCloud->header.frame_id;
      _grid2CloudPub.publish(grid2PassThroughCloud);
    }
    if ( _grid3CloudPub.getNumSubscribers() > 0 )
    {
      grid3PassThroughCloud->header.stamp    = pclDepthRGBCloud->header.stamp;
      grid3PassThroughCloud->header.frame_id = pclDepthRGBCloud->header.frame_id;
      _grid3CloudPub.publish(grid3PassThroughCloud);
    }
    if ( _grid4CloudPub.getNumSubscribers() > 0 )
    {
      grid4PassThroughCloud->header.stamp    = pclDepthRGBCloud->header.stamp;
      grid4PassThroughCloud->header.frame_id = pclDepthRGBCloud->header.frame_id;
      _grid4CloudPub.publish(grid4PassThroughCloud);
    }

    //Compute mean
    //std::cout << "Computing mean..." << std::endl;

    double max_z = 1.0;
    double sum = 0;
    double point;
    for(int i=0; i<pclDepthRGBCloud->size(); i++){
      point = (pclDepthRGBCloud->at(i).z - min_z_canon)/(max_z-min_z_canon); //Normalise point
      sum = point + sum;
      //sum = pclDepthRGBCloud->at(i).z + sum;
    }
    double mean = sum/pclDepthRGBCloud->size();
    //std::cout << "TOTAL mean: " << mean << std::endl;

    sum = 0;
    for(int i=0; i<grid1PassThroughCloud->size(); i++){
      point = (grid1PassThroughCloud->at(i).z - min_z_canon)/(max_z-min_z_canon); //Normalise point
      sum = point + sum;
      //sum = grid1PassThroughCloud->at(i).z + sum;
      //std::cout << grid1PassThroughCloud->at(i).x << std::endl;
    }
    if(grid1PassThroughCloud->size() < grid1_size)
    {
     // std::cout << "Null points exist in Grid 1" << std::endl;
      int dif = grid1_size - grid1PassThroughCloud->size();
      sum = sum + dif*1;
    }
    float mean1 = sum/grid1_size;
    //float mean1 = sum/grid1PassThroughCloud->size();
    //std::cout << "SUM: " << sum << std::endl;
    //std::cout << "Size: " << grid1PassThroughCloud->size() << std::endl;
    //std::cout << "GRID 1 mean: " << mean1 << std::endl;

    sum = 0;
    for(int i=0; i<grid2PassThroughCloud->size(); i++){
      point = (grid2PassThroughCloud->at(i).z - min_z_canon)/(max_z-min_z_canon); //Normalise point
      sum = point + sum;
      //sum = grid2PassThroughCloud->at(i).z + sum;
      //std::cout << grid1PassThroughCloud->at(i).x << std::endl;
    }
    if(grid2PassThroughCloud->size() < grid2_size)
    {
      //std::cout << "Null points exist in Grid 2" << std::endl;
      int dif = grid2_size - grid2PassThroughCloud->size();
      sum = sum + dif*1;
    }
    float mean2 = sum/grid2_size;
    //float mean2 = sum/grid2PassThroughCloud->size();
    //std::cout << "GRID 2 mean: " << mean2 << std::endl;

    sum = 0;
    for(int i=0; i<grid3PassThroughCloud->size(); i++){
      point = (grid3PassThroughCloud->at(i).z - min_z_canon)/(max_z-min_z_canon); //Normalise point
      sum = point + sum;
      //sum = grid3PassThroughCloud->at(i).z + sum;
      //std::cout << grid1PassThroughCloud->at(i).x << std::endl;
    }
    if(grid3PassThroughCloud->size() < grid3_size)
    {
     // std::cout << "Null points exist in Grid 3" << std::endl;
      int dif = grid3_size - grid3PassThroughCloud->size();
      sum = sum + dif*1;
    }
    float mean3 = sum/grid3_size;
    //float mean3 = sum/grid3PassThroughCloud->size();
    //std::cout << "GRID 3 mean: " << mean3 << std::endl;

    sum = 0;
    for(int i=0; i<grid4PassThroughCloud->size(); i++){
      point = (grid4PassThroughCloud->at(i).z - min_z_canon)/(max_z-min_z_canon); //Normalise point
      sum = point + sum;
      //sum = grid4PassThroughCloud->at(i).z + sum;
      //std::cout << grid1PassThroughCloud->at(i).x << std::endl;
    }
    if(grid4PassThroughCloud->size() < grid4_size)
    {
      //std::cout << "Null points exist in Grid 4" << std::endl;
      int dif = grid4_size - grid4PassThroughCloud->size();
      sum = sum + dif*1;
    }
    float mean4 = sum/grid4_size;
    //float mean4 = sum/grid4PassThroughCloud->size();
    // std::cout << "GRID 4 mean: " << mean4 << std::endl;

    //  v = grid1PassThroughCloud;
      //float average = accumulate( grid1PassThroughCloud->begin(), grid1PassThroughCloud->end(), 0.0/ grid1PassThroughCloud->size());
      //std::cout << "The average is" << average << std::endl;
    //std::cout << grid1PassThroughCloud->begin() << std::endl;

    visualization_msgs::Marker grid_marker1;
    visualization_msgs::Marker grid_marker2;
    visualization_msgs::Marker mean_marker1;
    visualization_msgs::Marker mean_marker2;
    visualization_msgs::Marker mean_marker3;
    visualization_msgs::Marker mean_marker4;

    //Initialize characteristics of the markers
    grid_marker1.header.frame_id = grid1PassThroughCloud->header.frame_id;
    grid_marker1.id = 1;
    grid_marker1.type = visualization_msgs::Marker::CUBE;
    grid_marker1.scale.x=0.25;
    grid_marker1.scale.y=0.005;
    grid_marker1.scale.z=0.005;
    grid_marker1.color.r = 1.0f;
    grid_marker1.color.g = 1.0f;
    grid_marker1.color.b = 1.0f;
    grid_marker1.color.a = 1.0;
    grid_marker1.lifetime = ros::Duration();
    grid_marker1.id=1;
    grid_marker1.pose.position.x=x_thr_canon; //x_threshold;
    grid_marker1.pose.position.y=y_thr_canon; //y_threshold;
    grid_marker1.pose.position.z=minPt.z;

    grid_marker2 = grid_marker1;
    grid_marker2.id=2;
    grid_marker2.scale.x=0.005;
    grid_marker2.scale.y=0.25;
    grid_marker2.scale.z=0.005;
    grid_marker2.pose.position.x=x_thr_canon; //x_threshold;
    grid_marker2.pose.position.y=y_thr_canon; //y_threshold;
    grid_marker2.pose.position.z=minPt.z;

    mean_marker1=grid_marker1;
    mean_marker1.id=3;
    mean_marker1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mean_marker1.text=std::to_string(mean1);
    mean_marker1.scale.z=0.02;
    mean_marker1.pose.position.x=x_threshold+0.15;
    mean_marker1.pose.position.y=y_threshold+0.15;
    mean_marker1.pose.position.z=minPt.z;

    mean_marker2=mean_marker1;
    mean_marker2.id=4;
    mean_marker2.text=std::to_string(mean2);
    mean_marker2.pose.position.x=x_threshold-0.15;

    mean_marker3=mean_marker1;
    mean_marker3.id=5;
    mean_marker3.text=std::to_string(mean3);
    mean_marker3.pose.position.y=y_threshold-0.15;

    mean_marker4=mean_marker1;
    mean_marker4.id=6;
    mean_marker4.text=std::to_string(mean4);
    mean_marker4.pose.position.x=x_threshold-0.15;
    mean_marker4.pose.position.y=y_threshold-0.15;


    visualization_msgs::MarkerArray means_markers;
    means_markers.markers.push_back(grid_marker1);
    means_markers.markers.push_back(grid_marker2);
    means_markers.markers.push_back(mean_marker1);
    means_markers.markers.push_back(mean_marker2);
    means_markers.markers.push_back(mean_marker3);
    means_markers.markers.push_back(mean_marker4);

    //Publish means
    if ( _metricMarkersPub.getNumSubscribers() > 1 )
    {
      _metricMarkersPub.publish(means_markers);
      means_markers.markers.clear();
    }

  }*/


  void SegmentPlane::placeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& placeCloud)
  {
    ROS_DEBUG("SegmentPlace: placeCloud Callback");
    if ( (placeCloud->width * placeCloud->height) == 0)
    {
      ROS_DEBUG("SegmentPlace: Empty placeCloud");
      return;
    }

    // Transform the point cloud to the frame specified if any
    sensor_msgs::PointCloud2Ptr newPlaceCloud;
    if ( !_processingFrame.empty() )
    {
      newPlaceCloud.reset(new sensor_msgs::PointCloud2);
      ROS_DEBUG_STREAM("Transforming Place point cloud from frame " << placeCloud->header.frame_id << " to frame " << _processingFrame);
      pcl_ros::transformPointCloud(_processingFrame, *placeCloud, *newPlaceCloud, _tfListener);
      newPlaceCloud->header.frame_id = _processingFrame;
    }
    else
      *newPlaceCloud = *placeCloud;

    // Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*newPlaceCloud, *pclCloud);

    // GET PILE HEIGHT
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*pclCloud, minPt, maxPt);
    std_msgs::Float64 pile_height;
    pile_height.data = minPt.x;
    _pileHeightPub.publish(pile_height);

    // GET PILE CORNERS
    visualization_msgs::MarkerArray corners_markers;
    corners_markers = computeCorners(pclCloud);

    //Publish corners
    _placeCornersMarkersPub.publish(corners_markers);
	  corners_markers.markers.clear();

  }

  void SegmentPlane::pickCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pickCloud)
  {
    ROS_DEBUG("SegmentPlace: pickCloud Callback");

    // To get the segmented cloud in PCL XYZ format (instead of XYZRGB)
    if ( (pickCloud->width * pickCloud->height) == 0)
    {
      ROS_DEBUG("SegmentPlace: Empty pickCloud");
      return;
    }

    sensor_msgs::PointCloud2Ptr xyzcloudInProcFrame;

    // Transform the point cloud to the frame specified if any
    if ( !_processingFrame.empty() )
    {
      xyzcloudInProcFrame.reset(new sensor_msgs::PointCloud2);
      ROS_DEBUG_STREAM("Transforming Pick point cloud from frame " << pickCloud->header.frame_id << " to frame " << _processingFrame);
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

    // pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    // pcl::copyPointCloud(cloud_xyz, pclFilteredNonPlaneCloud);

    /*pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud_hull, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;*/

    // GET GARMENT CORNERS
    visualization_msgs::MarkerArray corners_markers;
    corners_markers = computeCorners(cloud_hull);

    
    //Publish corners
    if ( _cornersMarkersPub.getNumSubscribers() > 1 )
    {
      _cornersMarkersPub.publish(corners_markers);
	    corners_markers.markers.clear();
    }



    //Robust corner points - sliding window

    /*std::cout << "\043[1;36m PT UP RIGHT: " << pt_up_right.z << ", " << pt_up_right.y << ", " << pt_up_right.x << std::endl;
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
    /*    // Compute edges size
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

      //  std_msgs::Float64 grasp_angle;
      //  grasp_angle.data = alpha;
        _graspPointAnglePub.publish(grasp_angle);
      */
    //  }

  }

  //Detects corners of pointcloud with sum/diff of min/max points
  visualization_msgs::MarkerArray SegmentPlane::computeCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr garmentCloud)
  {
    ROS_DEBUG("SegmentPlane: Computing corners");
    
    // Create markers to visualise corners in RVIZ
    visualization_msgs::Marker corner1_marker;
    visualization_msgs::Marker corner2_marker;
    visualization_msgs::Marker corner3_marker;
    visualization_msgs::Marker corner4_marker;
    visualization_msgs::MarkerArray corners_markers;

    //Initialize characteristics of the markers
    corner1_marker.header.frame_id = garmentCloud->header.frame_id;
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

    // GET CORNERS
    pcl::PointXYZ pt_down_right, pt_down_left, pt_up_right, pt_up_left;
    float max_diff = garmentCloud->points[0].y - garmentCloud->points[0].z;
    float min_sum = garmentCloud->points[0].y + garmentCloud->points[0].z;
    float min_diff = max_diff;
    float max_sum = min_sum;
    //std::cout << "max diff: " << max_diff << "min sum: " << min_sum << std::endl;

    //Init
    pt_down_right.x=garmentCloud->points[0].x;
    pt_down_right.y=garmentCloud->points[0].y;
    pt_down_right.z=garmentCloud->points[0].z;
    pt_up_left.x=garmentCloud->points[0].x;
    pt_up_left.y=garmentCloud->points[0].y;
    pt_up_left.z=garmentCloud->points[0].z;
    pt_up_right.x=garmentCloud->points[0].x;
    pt_up_right.y=garmentCloud->points[0].y;
    pt_up_right.z=garmentCloud->points[0].z;
    pt_down_left.x=garmentCloud->points[0].x;
    pt_down_left.y=garmentCloud->points[0].y;
    pt_down_left.z=garmentCloud->points[0].z;

    //Get corners
    for(int i=0; i<garmentCloud->points.size(); i++)
    {
      float diff = garmentCloud->points[i].y - garmentCloud->points[i].z;
      float sum = garmentCloud->points[i].y + garmentCloud->points[i].z;
      if(min_sum > sum)
      {
          min_sum=sum;
          pt_up_left.x=garmentCloud->points[i].x;
          pt_up_left.y=garmentCloud->points[i].y;
          pt_up_left.z=garmentCloud->points[i].z;
      }
      if(max_sum < sum)
      {
          max_sum=sum;
          pt_down_right.x=garmentCloud->points[i].x;
          pt_down_right.y=garmentCloud->points[i].y;
          pt_down_right.z=garmentCloud->points[i].z;
      }
      if(min_diff > diff)
      {
          min_diff=diff;
          pt_down_left.x=garmentCloud->points[i].x;
          pt_down_left.y=garmentCloud->points[i].y;
          pt_down_left.z=garmentCloud->points[i].z;
      }
      if(max_diff < diff)
      {
          max_diff=diff;
          pt_up_right.x=garmentCloud->points[i].x;
          pt_up_right.y=garmentCloud->points[i].y;
          pt_up_right.z=garmentCloud->points[i].z;
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

    // //Publish corners
    // if ( _placeCornersMarkersPub.getNumSubscribers() > 1 )
    // {
    //   _placeCornersMarkersPub.publish(corners_markers);
	  //   corners_markers.markers.clear();
    // }

    return corners_markers;

  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentPlane::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud, std::string axis, double min, double max)
  {
    // Apply passthrough filter for Pick zone
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( !axis.empty() )
    {
      pal::passThrough<pcl::PointXYZRGB>(pclCloud,
                                         axis,
                                         min, max,
                                         passThroughCloud);

      // if ( passThroughCloud->empty() )
      // {
      //    //if all points get removed after the pass-through filtering just publish empty point clouds
      //    //and stop processing
      //    publishEmptyClouds(pclCloud->header.stamp, pclCloud->header.frame_id);
      //    return;
      // }
    }
    else
      passThroughCloud = pclCloud;
  
    return passThroughCloud;
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
    //_garmentCloudSub = _nh.subscribe("garmentcloud", 1, &SegmentPlane::garmentCloudCallback, this);
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
