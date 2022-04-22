// Copyright (C) 2010-2011 Institut de Robotita i Informatica Industrial, CSIC-UPC.
// Author Irene Garcia-Camacho
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _pick_garment_demo_alg_node_h_
#define _pick_garment_demo_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "pick_garment_demo_alg.h"

//IRI ROS headers
#include <iri_ros_tools/timeout.h>

//Modules headers
#include <tiago_modules/gripper_module.h>
#include <tiago_modules/arm_module.h>
#include <tiago_modules/torso_module.h>
#include <tiago_modules/head_module.h>
#include <tiago_modules/play_motion_module.h>
#include <tiago_modules/move_platform_module.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

//Main state machine
typedef enum {IDLE,
	      LOOK_AROUND,
	      LOOK_DOWN,
          GET_GRASP_TARGETS,
	      WAIT_PREGRASP,
	      WAIT_PREGRASP2,
          WAIT_GRASP,
	      CLOSE_GRIPPER,
          WAIT_POSTGRASP,
          PREGIVE,
          GIVE_GARMENT,
          SET_POSITION,
          MOVE_PLATFORM,
          FINAL_POSITION,
          OPEN_GRIPPER,
	      END} pick_garment_states_t;

typedef enum{SUCCESS,
    	     RUNNING,
	         CANCELED,
             NO_TRANSFORM,
	         ERROR}pick_garment_status_t;


/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PickGarmentDemoAlgNode : public algorithm_base::IriBaseAlgorithm<PickGarmentDemoAlgorithm>
{
  private:

// Garment pose subscriber
    ros::Subscriber garment_pose_subscriber;
    void garment_pose_callback(const visualization_msgs::Marker::ConstPtr& msg);

// Grasping position marker publisher
    ros::Publisher marker_pose_publisher;
    ros::Timer marker_pose_timer;

// Fingertip frame publisher rate
    ros::Timer new_frames_pub_timer;

// Modules
    CGripperModule gripper;

    CArmModule arm;
    geometry_msgs::PoseStamped garment_pose;
    
    geometry_msgs::PoseStamped pregrasp_arm_target;
    geometry_msgs::PoseStamped grasp_arm_target;
    geometry_msgs::PoseStamped postgrasp_arm_target;
    geometry_msgs::PoseStamped grasp_pose;

    CTorsoModule torso;
    double torso_position;

    CHeadModule head;
    std::vector<double> pan_angles, tilt_angles;
    std::vector<double> durations;
    double head_duration;

    CPlayMotionModule play_motion;
    
    CMovePlatformModule move_platform;

// Variables
    pick_garment_states_t state;
    pick_garment_status_t status;
    
    bool start;
    bool start_platform;
    bool start_give;
    bool stop;

    bool get_garment_pose;

    CROSTimeout tf_timeout;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PickGarmentDemoAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PickGarmentDemoAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

   /**
    * \brief Publishers rate configuration
    * 
    * Sets the rates for "marker_pose_pub" and "new_frames_pub" functions, 
    * corresponding to the rates for publishing the grasping position marker 
    * and the fingertip frame. 
    */
    void set_config(void);

   /**
    * \brief Grasping position marker publisher
    * 
    * Publishes the grasping position for visualization purposes as visualization_msg/Marker.
    * Can be visualized with RVIZ.
    */
    void marker_pose_pub(const ros::TimerEvent& event);

   /**
    * \brief Fingertip frame publisher
    *
    * Creates and publishes a new frame corresponding to the fingertip.
    */    
    void new_frames_pub(const ros::TimerEvent& event);

   /**
    * \brief Scan head angles
    * 
    * This function genereates the pan and tilt angles to perdorm a scan 
    * of the environment in order to obtain a complete the octomap.
    */
    void set_head_scan_angles(void);

   /**
    * \brief computes grasp targets
    *
    * This function computes the pregrasp, grasp and postgrasp positions 
    * in reference of /arm_tool_link in order to have the fingertip 
    * (represented by /grasp_frame) in the garment's position.
    */
    bool compute_grasp_arm_targets(void);

};

#endif
