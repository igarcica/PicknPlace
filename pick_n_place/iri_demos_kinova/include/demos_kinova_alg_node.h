// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author
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

#ifndef _demos_kinova_alg_node_h_
#define _demos_kinova_alg_node_h_
#define HOME_ACTION_IDENTIFIER 2

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "demos_kinova_alg.h"
#include <kortex_driver/Pose.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionEvent.h>
#include <thread>
#include <tf/transform_datatypes.h>

// [publisher subscriber headers]
#include <kortex_driver/TwistCommand.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ActionNotification.h>

// [service client headers]
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/OnNotificationActionTopic.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_kinova_linear_movement/kinova_linear_movementAction.h>
#include <iri_action_server/iri_action_server.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

typedef enum {IDLE,
              HOME,
              PRE_GRASP,
              GRASP,
              WAIT_GRASP,
              CLOSE_GRIPPER,
              POST_GRASP,
              ROTATE_POST_GRASP,
              WAIT_POST_GRASP,
              GO_TO_PLACE,
              WAIT_GO_TO_PLACE,
              ROTATE_PRE_PLACE,
              PRE_PLACE_DIAGONAL,
              WAIT_PRE_PLACE_DIAGONAL,
              PLACE_DIAGONAL,
              WAIT_PLACE_DIAGONAL,
              PRE_PLACE2,
              PLACE2,
              PLACE22,
              PLACE_RECTO,
              WAIT_PLACE_RECTO,
	      PILING,
	      PILING2,
              OPEN_GRIPPER,
              POST_PLACE,
              WAIT_POST_PLACE,
              HIGH_POSITION,
              WAIT_HIGH_POSITION,
              END} pick_place_states_t;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class DemosKinovaAlgNode : public algorithm_base::IriBaseAlgorithm<DemosKinovaAlgorithm>
{
  private:
    std::string robot_name;
    std::atomic<int> last_action_notification_event{0};
    kortex_driver::Pose tool_pose;
    bool success = true;
    //int state = 0;
    bool start=false;
    bool stop=false;
    pick_place_states_t state;
    double close_gripper;
    double open_gripper;

    std::vector<double> pre_grasp_corner;
    kortex_driver::Pose pre_grasp_center;
    kortex_driver::Pose grasping_point_garment;
    float garment_width;
    float garment_height;
    int cartesian_rf;
    bool diagonal_move;
    bool clear_faults(void);
    bool set_cartesian_reference_frame(const int &cartesian_rf);
    bool send_gripper_command(double value);
    bool home_the_robot(void);
    bool send_cartesian_pose(const kortex_driver::Pose &goal_pose);
    bool wait_for_action_end_or_abort(void);
    kortex_driver::Waypoint FillCartesianWaypoint(const kortex_driver::Pose &goal_pose, float blending_radius);

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    ros::Timer handeye_frame_pub_timer;
    void handeye_frame_pub(const ros::TimerEvent& event);
    geometry_msgs::PoseStamped grasp_pose;
    bool get_garment_position;
    bool get_garment_angle;
    ros::Subscriber garment_pose_subscriber;
    //ros::Subscriber garment_angle_subscriber;
    ros::Subscriber garment_edge_subscriber;
    ros::Subscriber corners_subscriber;
    void garment_pose_callback(const visualization_msgs::Marker::ConstPtr& msg);
    //void garment_angle_callback(const std_msgs::Float64::ConstPtr& msg);
    void compute_grasp_angle(const std_msgs::Float64& msg);
    void garment_edge_callback(const std_msgs::Float64::ConstPtr& msg);
    //void select_grasp_point();
    void corners_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void get_params(void);
    kortex_driver::Pose pre_grasp_distance;

    ros::Publisher garment_marker_publisher;
    ros::Publisher grasp_marker_publisher;

    // [publisher attributes]
    ros::Publisher cartesian_velocity_publisher_;
    kortex_driver::TwistCommand cartesian_velocity_TwistCommand_msg_;


    // [subscriber attributes]
    ros::Subscriber base_feedback_subscriber_;
    void base_feedback_callback(const kortex_driver::BaseCyclic_Feedback::ConstPtr& msg);
    pthread_mutex_t base_feedback_mutex_;
    void base_feedback_mutex_enter(void);
    void base_feedback_mutex_exit(void);

    ros::Subscriber action_topic_subscriber_;
    void action_topic_callback(const kortex_driver::ActionNotification::ConstPtr& msg);
    pthread_mutex_t action_topic_mutex_;
    void action_topic_mutex_enter(void);
    void action_topic_mutex_exit(void);


    // [service attributes]

    // [client attributes]
    ros::ServiceClient exec_wp_trajectory_client_;
    kortex_driver::ExecuteWaypointTrajectory exec_wp_trajectory_srv_;

    ros::ServiceClient base_execute_action_client_;
    kortex_driver::ExecuteAction base_execute_action_srv_;

    ros::ServiceClient base_read_action_client_;
    kortex_driver::ReadAction base_read_action_srv_;

    ros::ServiceClient send_gripper_cmd_client_;
    kortex_driver::SendGripperCommand send_gripper_cmd_srv_;

    ros::ServiceClient set_cartesian_rf_client_;
    kortex_driver::SetCartesianReferenceFrame set_cartesian_rf_srv_;

    ros::ServiceClient base_clear_faults_client_;
    kortex_driver::Base_ClearFaults base_clear_faults_srv_;

    ros::ServiceClient activate_publishing_client_;
    kortex_driver::OnNotificationActionTopic activate_publishing_srv_;


    // [action server attributes]

    // [action client attributes]
    actionlib::SimpleActionClient<iri_kinova_linear_movement::kinova_linear_movementAction> kinova_linear_move_client_;
    iri_kinova_linear_movement::kinova_linear_movementGoal kinova_linear_move_goal_;
    bool kinova_linear_moveMakeActionRequest(const geometry_msgs::Pose& desired_pose, const int& rf_frame, const float& max_vel);
    void kinova_linear_moveDone(const actionlib::SimpleClientGoalState& state,  const iri_kinova_linear_movement::kinova_linear_movementResultConstPtr& result);
    void kinova_linear_moveActive();
    void kinova_linear_moveFeedback(const iri_kinova_linear_movement::kinova_linear_movementFeedbackConstPtr& feedback);



   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    DemosKinovaAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~DemosKinovaAlgNode(void);

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

    // [diagnostic functions]

    // [test functions]
};

#endif
