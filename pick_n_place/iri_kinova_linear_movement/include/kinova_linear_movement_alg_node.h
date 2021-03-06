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

#ifndef _kinova_linear_movement_alg_node_h_
#define _kinova_linear_movement_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "kinova_linear_movement_alg.h"
#include <kortex_driver/Pose.h>
#include <tf/transform_datatypes.h>

// [publisher subscriber headers]
#include <kortex_driver/TwistCommand.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <iri_kinova_linear_movement/kinova_linear_movementAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class KinovaLinearMovementAlgNode : public algorithm_base::IriBaseAlgorithm<KinovaLinearMovementAlgorithm>
{
  private:
    std::string robot_name;
    kortex_driver::Pose tool_pose;
    kortex_driver::TwistCommand twist_cmd;
    tf::Vector3 goal_pos;
    ros::Time start_time;
    float action_time_limit;
    ros::Duration total_time;
    float maximum_vel;
    bool stop_robot = false;
    void move_to_pos(const iri_kinova_linear_movement::kinova_linear_movementGoalConstPtr& goal);
    //kortex_driver::TwistCommand get_velocity(const tf::Vector3& error, const tf::Vector3& ac_error);
    kortex_driver::TwistCommand get_velocity(const tf::Vector3& error);
    // [publisher attributes]
    ros::Publisher cartesian_velocity_publisher_;
    kortex_driver::TwistCommand cartesian_velocity_TwistCommand_msg_;


    // [subscriber attributes]
    ros::Subscriber base_feedback_subscriber_;
    void base_feedback_callback(const kortex_driver::BaseCyclic_Feedback::ConstPtr& msg);
    pthread_mutex_t base_feedback_mutex_;
    void base_feedback_mutex_enter(void);
    void base_feedback_mutex_exit(void);


    // [service attributes]

    // [client attributes]

    // [action server attributes]
    IriActionServer<iri_kinova_linear_movement::kinova_linear_movementAction> kinova_linear_move_aserver_;
    void kinova_linear_moveStartCallback(const iri_kinova_linear_movement::kinova_linear_movementGoalConstPtr& goal);
    void kinova_linear_moveStopCallback(void);
    bool kinova_linear_moveIsFinishedCallback(void);
    bool kinova_linear_moveHasSucceededCallback(void);
    void kinova_linear_moveGetResultCallback(iri_kinova_linear_movement::kinova_linear_movementResultPtr& result);
    void kinova_linear_moveGetFeedbackCallback(iri_kinova_linear_movement::kinova_linear_movementFeedbackPtr& feedback);
    bool kinova_linear_move_active;
    bool kinova_linear_move_succeeded;
    bool kinova_linear_move_finished;


    // [action client attributes]

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
    KinovaLinearMovementAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~KinovaLinearMovementAlgNode(void);

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
