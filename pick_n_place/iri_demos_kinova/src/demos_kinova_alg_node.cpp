#include "demos_kinova_alg_node.h"

DemosKinovaAlgNode::DemosKinovaAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DemosKinovaAlgorithm>(),
  kinova_linear_move_client_(private_node_handle_,"kinova_linear_move", true)
{

  this->state=IDLE;
  this->start=false;
  this->stop=false;
  double open_gripper = 0.35;
  double close_gripper = 0.97; //0.81;

  // Garment pose subscriber
  this->garment_pose_subscriber = this->public_node_handle_.subscribe("/segment_table/grasp_point",1,&DemosKinovaAlgNode::garment_pose_callback,this);
  this->get_garment_position=false;
  this->get_garment_angle=false;
  //this->garment_angle_subscriber = this->public_node_handle_.subscribe("/segment_table/grasp_angle",1,&DemosKinovaAlgNode::garment_angle_callback,this);
  this->garment_edge_subscriber = this->public_node_handle_.subscribe("/segment_table/garment_edge",1,&DemosKinovaAlgNode::garment_edge_callback,this);
  this->corners_subscriber = this->public_node_handle_.subscribe("/segment_table/corners",1,&DemosKinovaAlgNode::corners_callback,this);

  // Publish grasp marker
  this->garment_marker_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("garment_marker", 1);
  this->grasp_marker_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("grasp_marker", 1);

  // Publish handeye transform between ext_camera_link to base_link
  this->handeye_frame_pub_timer = this->public_node_handle_.createTimer(ros::Duration(1.0),&DemosKinovaAlgNode::handeye_frame_pub,this);
  this->handeye_frame_pub_timer.setPeriod(ros::Duration(1.0/50.0));
  this->handeye_frame_pub_timer.start();
  //this->handeye_frame_pub_timer.stop();

  this->get_params();

  this->pre_grasp_center.x = this->pre_grasp_corner[0]-0.08;
  this->pre_grasp_center.y = this->pre_grasp_corner[1];// - this->garment_width/2;
  this->pre_grasp_center.z = this->pre_grasp_corner[2];
  this->pre_grasp_center.theta_x = this->pre_grasp_corner[3];
  this->pre_grasp_center.theta_y = this->pre_grasp_corner[4];
  this->pre_grasp_center.theta_z = this->pre_grasp_corner[5];

  // [init publishers]
  this->cartesian_velocity_publisher_ = this->private_node_handle_.advertise<kortex_driver::TwistCommand>("/" + this->robot_name + "/in/cartesian_velocity", 1);

  // [init subscribers]
  this->base_feedback_subscriber_ = this->private_node_handle_.subscribe("/" + this->robot_name  + "/base_feedback", 1000, &DemosKinovaAlgNode::base_feedback_callback, this);
  pthread_mutex_init(&this->base_feedback_mutex_,NULL);

  this->action_topic_subscriber_ = this->private_node_handle_.subscribe("/" + this->robot_name  + "/action_topic", 1000, &DemosKinovaAlgNode::action_topic_callback, this);
  pthread_mutex_init(&this->action_topic_mutex_,NULL);


  // [init services]

  // [init clients]
  exec_wp_trajectory_client_ = this->private_node_handle_.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/" + robot_name + "/base/execute_waypoint_trajectory");

  base_execute_action_client_ = this->private_node_handle_.serviceClient<kortex_driver::ExecuteAction>("/" + this->robot_name + "/base/execute_action");

  base_read_action_client_ = this->private_node_handle_.serviceClient<kortex_driver::ReadAction>("/" + this->robot_name + "/base/read_action");

  send_gripper_cmd_client_ = this->private_node_handle_.serviceClient<kortex_driver::SendGripperCommand>("/" + this->robot_name + "/base/send_gripper_command");

  set_cartesian_rf_client_ = this->private_node_handle_.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + this->robot_name + "/control_config/set_cartesian_reference_frame");

  base_clear_faults_client_ = this->private_node_handle_.serviceClient<kortex_driver::Base_ClearFaults>("/" + this->robot_name + "/base/clear_faults");

  activate_publishing_client_ = this->private_node_handle_.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + this->robot_name + "/base/activate_publishing_of_action_topic");

  // [init action servers]

  // [init action clients]


  ROS_INFO("DemosKinovaAlgNode:: Calling service activate_publishing_client_!");
  if (activate_publishing_client_.call(activate_publishing_srv_))
  {
    ROS_INFO("Action notification activated!");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->activate_publishing_client_.getService().c_str());
    this->success = false;
  }

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  this->success &= clear_faults();
  if (!this->success) exit(1);
  //*******************************************************************************

  this->success &= set_cartesian_reference_frame(kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED);
  if (!this->success) exit(1);
}

DemosKinovaAlgNode::~DemosKinovaAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->base_feedback_mutex_);
  pthread_mutex_destroy(&this->action_topic_mutex_);
}

void DemosKinovaAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread");
  // [fill msg structures]
  // Initialize the topic message structure
  //this->cartesian_velocity_TwistCommand_msg_.data = my_var;

  // Initialize the topic message structure
  //this->my_gen3_action_topic_ActionNotification_msg_.data = my_var;


  // [fill srv structure and make request to the server]


  //activate_publishing_srv_.request.data = my_var;


  if(this->state!=IDLE && this->stop)
  {
    ROS_WARN("Demo Pick n Place has stopped!");
    this->state=IDLE;
    this->start=false;
    this->stop=false;
  }
  else
  {
    switch(this->state)
    {
      case IDLE: ROS_DEBUG("DemosKinovaAlgNode: state IDLE");
                 if(this->start)
                 {
                   ROS_DEBUG("Opening the gripper.");
                   this->success &= send_gripper_command(this->open_gripper);
                   if (this->success)
                   {
                     this->state=HOME;
                     ros::Duration(0.5).sleep();
                     this->start=false;
                   }
                 }
                 else
                   this->state=IDLE;
      break;

      // HOME POSITION
      case HOME: ROS_DEBUG("DemosKinovaAlgNode: state HOME");
                 // Move the robot to the Home position with an Action
                 ROS_INFO("Moving to home position.");
                 this->success &= home_the_robot();
                 if (this->success)
                 {
                   this->state=PRE_GRASP;
                   ros::Duration(0.5).sleep();
                 }
      break;

      // PRE-GRASP POSITION
      case PRE_GRASP: ROS_DEBUG("DemosKinovaAlgNode: state PRE GRASP");
                      ROS_INFO("Sending to pre-grasp position.");
                      std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->grasping_point_garment.x << ", y: " << this->grasping_point_garment.y << ", z: " << this->grasping_point_garment.z << std::endl;
                      this->success &= send_cartesian_pose(this->grasping_point_garment);
                      if (this->success)
                      {
                        ROS_INFO("Success PRE GRASP");
                        this->state=GRASP;
                        ros::Duration(0.5).sleep();
                      }
      break;


      // GRASP POSITION
      case GRASP: ROS_DEBUG("DemosKinovaAlgNode: state GRASP");
                  {
                      ROS_INFO("Sending to grasp position.");
                      geometry_msgs::Pose desired_pose;
                      desired_pose.position.x = tool_pose.x + this->pre_grasp_distance.x;  // + 0.05;
                      desired_pose.position.y = tool_pose.y + this->pre_grasp_distance.y;
                      desired_pose.position.z = tool_pose.z;
                      std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.1);
                      this->state=WAIT_GRASP;
                  }
      break;

      case WAIT_GRASP: ROS_DEBUG("DemosKinovaAlgNode: state WAIT GRASP");
                       {
                         actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                         // to get the state of the current goal
                         this->alg_.unlock();
                         kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                         this->alg_.lock();

                         ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
			 // falta un timeout (state LOST)
                         if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                         {
                           ROS_INFO("Action aborted!");
                           this->state=END; //
                         }
                         else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                         {
                           this->success = true;
                           this->state=CLOSE_GRIPPER;
                           ros::Duration(0.5).sleep();
                         }
                       }
      break;


      // CLOSE GRIPPER
      case CLOSE_GRIPPER: ROS_DEBUG("DemosKinovaAlgNode: state CLOSE GRIPPER");
                          ROS_INFO("Closing the gripper.");
                          this->success &= send_gripper_command(this->close_gripper);
                          if (this->success)
                          {
                            this->state=POST_GRASP;
                            ros::Duration(0.5).sleep();
                          }
      break;


      // POST-GRASP POSITION
      // Sets a post grasp position a bit (x1.2) more high than the width of the garment
      case POST_GRASP: ROS_DEBUG("DemosKinovaAlgNode: state POST GRASP");
                       {
                           ROS_INFO("Sending to post-grasp position.");
                           geometry_msgs::Pose desired_pose;
                           desired_pose.position.x = tool_pose.x;
                           desired_pose.position.y = tool_pose.y;
                           desired_pose.position.z = this->garment_height*1.2;
                           std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                           kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                           this->state=WAIT_POST_GRASP;
                       }
      break;

      // Waits until it reaches the post grasp position (linear movement controller)
      case WAIT_POST_GRASP: ROS_DEBUG("DemosKinovaAlgNode: state WAIT POST GRASP");
                            {
                              actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                              // to get the state of the current goal
                              this->alg_.unlock();
                              kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                              this->alg_.lock();

                              ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                              if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                              {
                                ROS_INFO("Action aborted!");
                                this->state=END; //
                              }
                              else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                              {
                                this->success = true;
                                this->state=ROTATE_POST_GRASP;
                                ros::Duration(0.5).sleep();
                              }
                            }
      break;

      // ROTATE POST-GRASP POSITION - CARTESIAN
      // Sets a position under the camera with an horizontal orientation to check the deformation
      case ROTATE_POST_GRASP: ROS_DEBUG("DemosKinovaAlgNode: state ROTATE POST GRASP");
                              ROS_INFO("Rotating post-grasp position.");
                              this->pre_grasp_center.x = 0.5; //tool_pose.x;
                              this->pre_grasp_center.y = 0.08; //tool_pose.y;
                              this->pre_grasp_center.z = 0.30; //tool_pose.z;
                              this->pre_grasp_center.theta_x = 0.0; //0.0;
                              this->pre_grasp_center.theta_y = -90; //-125.5;
                              this->pre_grasp_center.theta_z = 180; //180;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                              this->success &= send_cartesian_pose(this->pre_grasp_center);
                              if (this->success)
                              {
                                ROS_INFO("Success ROTATE PRE GRASP");
                                this->state=GO_TO_PLACE;
                                ros::Duration(0.5).sleep();
                              }
      break;

      // PRE-PLACE POSITION
      // Go to a fixed pre place position
      case GO_TO_PLACE: ROS_DEBUG("DemosKinovaAlgNode: state GO TO PLACE");
                        {
                          if(config_.ok){
                            ROS_INFO("Sending to pre-place position.");
                            //first=false;
                            geometry_msgs::Pose desired_pose;
                            desired_pose.position.x = 0.8;
                            desired_pose.position.y = -0.1;//-0.28//0.0; //-0.12; //-this->garment_width/2;
                            desired_pose.position.z = tool_pose.z;
                            std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                            kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                            config_.ok=false;
                            this->state=WAIT_GO_TO_PLACE;
                          }
                          else
                            this->state=GO_TO_PLACE;
                        }
      break;

      // Waits until it reaches the pre place position with linear movement controller
      case WAIT_GO_TO_PLACE: ROS_DEBUG("DemosKinovaAlgNode: state WAIT GO TO PLACE");
                             {
                               actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                               // to get the state of the current goal
                               this->alg_.unlock();
                               kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                               this->alg_.lock();

                               ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                               if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                               {
                                 ROS_INFO("Action aborted!");
                                 this->state=END;
                               }
                               else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                               {
                                 this->success = true;
                                 if(this->diagonal_move)
                                   this->state=ROTATE_PRE_PLACE; //PRE_PLACE_DIAGONAL;
                                 else if(config_.vertical_place)
				   this->state=PLACE_RECTO;
                                   //this->state=PLACE;
				 else if(config_.place2)
                                   this->state=PRE_PLACE2;
                                 //else
                                 //  this->state=ROTATE_PRE_PLACE;
                                 ros::Duration(0.5).sleep();
                               }
                             }
      break;

      // ROTATE PRE-PLACE POSITION - CARTESIAN
      // Sets a slight rotation before the diagonal placement
      case ROTATE_PRE_PLACE: ROS_DEBUG("DemosKinovaAlgNode: state ROTATE PRE PLACE");
                              ROS_INFO("Rotating pre-place position.");
                              this->pre_grasp_center.x = tool_pose.x;
                              this->pre_grasp_center.y = tool_pose.y;
                              this->pre_grasp_center.z = this->garment_height+0.05;//*1.2; //Check;
                              this->pre_grasp_center.theta_x = 0.0;
                              this->pre_grasp_center.theta_y = -125.5;
                              this->pre_grasp_center.theta_z = 180;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                              this->success &= send_cartesian_pose(this->pre_grasp_center);
                              if (this->success)
                              {
                                ROS_INFO("Success ROTATE PRE PLACE");
                                //if(this->diagonal_move)
                                this->state=PLACE_DIAGONAL;
                                //else
                                //  this->state=PLACE_RECTO;
                                ros::Duration(0.5).sleep();
                              }
      break;

      // PLACE POSITION
      // Sets the placing position so it performs a diagonal movement
      case PLACE_DIAGONAL: ROS_DEBUG("DemosKinovaAlgNode: state PLACE DIAGONAL");
                           {
                               ROS_INFO("Sending to place position.");
                               geometry_msgs::Pose desired_pose;
                               desired_pose.position.x = tool_pose.x-this->garment_height;///1.5; //Check
                               desired_pose.position.y = tool_pose.y; //-0.3;//tool_pose.y;
                               desired_pose.position.z = 0.055; //this->pre_grasp_center.z;
                               std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                               kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
//                             }
                             this->state=WAIT_PLACE_DIAGONAL;
                           }
      break;

      // Wait until it ends the diagonal movement (with linear movement controller)
      case WAIT_PLACE_DIAGONAL: ROS_DEBUG("DemosKinovaAlgNode: state WAIT PLACE DIAGONAL");
                                {
                                  actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                                  // to get the state of the current goal
                                  this->alg_.unlock();
                                  kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                                  this->alg_.lock();

                                  ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                                  if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                                  {
                                    ROS_INFO("Action aborted!");
                                    this->state=END;
                                  }
                                  else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                                  {
                                    this->success = true;
                                    this->state=OPEN_GRIPPER;
                                    ros::Duration(0.5).sleep();
                                  }
                                }
      break;

//Place rotando
      // PRE PLACE2 - CARTESIAN
      // Sets a rotation so the garment stays vertical to the table (with cartesian controller)
      case PRE_PLACE2: ROS_DEBUG("DemosKinovaAlgNode: state ROTATE PRE PLACE2");
                              ROS_INFO("Rotating pre-place position.");
                              this->pre_grasp_center.x = 0.5; //tool_pose.x-this->garment_height/1.5;
                              this->pre_grasp_center.y = -0.28; //tool_pose.y;//-0.15;
			      this->pre_grasp_center.z = this->garment_height+0.04;
                              this->pre_grasp_center.theta_x = 0;
                              this->pre_grasp_center.theta_y = 175;
                              this->pre_grasp_center.theta_z = 179;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                              this->success &= send_cartesian_pose(this->pre_grasp_center);
                              if (this->success)
                              {
                                ROS_INFO("Success PLACE2");
                                ros::Duration(0.5).sleep();
				if(config_.piling)
				  this->state=PILING;
				else
                                  this->state=PLACE2;
                              }
			      else
			        this->state=END;
      break;

      // PLACE2 - CARTESIAN
      // Sets the placing position in the table from the vertical rotation (with cartesian controller)
      case PLACE2: ROS_DEBUG("DemosKinovaAlgNode: state ROTATE PRE PLACE");
                              ROS_INFO("Sending to place position.");
                              this->pre_grasp_center.x = tool_pose.x-this->garment_height-0.07;//*1.2;///1.5;
                              this->pre_grasp_center.y = tool_pose.y;
                              this->pre_grasp_center.z = 0.1; //0.055; //0.1;
                              this->pre_grasp_center.theta_x = 0;
                              this->pre_grasp_center.theta_y = -125.5;
                              this->pre_grasp_center.theta_z = 180;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                              this->success &= send_cartesian_pose(this->pre_grasp_center);
                              if (this->success)
                              {
                                ROS_INFO("Success PLACE2");
                                ros::Duration(0.5).sleep();
                              //  this->state=OPEN_GRIPPER;
			        this->state=PLACE22;
                              }
			      else
                              {
                                ROS_INFO("UNSUCCESS PLACE2");
                                ros::Duration(0.5).sleep();
                                //this->state=OPEN_GRIPPER;
			        this->state=PLACE22;
                              }
      break;

      case PLACE22: ROS_DEBUG("DemosKinovaAlgNode: state PLACE22");
                              ROS_INFO("Sending to place2 position.");
                              this->pre_grasp_center.x = tool_pose.x-0.07;
                              this->pre_grasp_center.y = tool_pose.y;
                              this->pre_grasp_center.z = 0.055;
                              this->pre_grasp_center.theta_x = 0;
                              this->pre_grasp_center.theta_y = -125.5;
                              this->pre_grasp_center.theta_z = 180;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                              this->success &= send_cartesian_pose(this->pre_grasp_center);
                              if (this->success)
                              {
                                ROS_INFO("Success PLACE2");
                                ros::Duration(0.5).sleep();
                                this->state=OPEN_GRIPPER;
                              }
			      else
                              {
                                ROS_INFO("UNSUCCESS PLACE2");
                                ros::Duration(0.5).sleep();
                                this->state=OPEN_GRIPPER;
                              }
			        //this->state=END;
      break;



// PLACE RECTO!

      // PLACE POSITION
      // Sets the placing position to perform a straight placement (with linear movement controller)
      case PLACE_RECTO: ROS_DEBUG("DemosKinovaAlgNode: state PLACE RECTO");
                  {
                      ROS_INFO("Sending to place position.");
                      geometry_msgs::Pose desired_pose;
                      desired_pose.position.x = tool_pose.x;
                      desired_pose.position.y = tool_pose.y;
                      desired_pose.position.z = 0.07;
                      std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                    this->state=WAIT_PLACE_RECTO;
                  }
      break;

      // Waits until it places the garment vertically (linear)
      case WAIT_PLACE_RECTO: ROS_DEBUG("DemosKinovaAlgNode: state WAIT PLACE RECTO");
                       {
                         actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                         // to get the state of the current goal
                         this->alg_.unlock();
                         kinova_linear_move_state=kinova_linear_move_client_.getState();
                         // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                         this->alg_.lock();
                         ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                         if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                         {
                           ROS_INFO("Action aborted!");
                           this->state=END;
                         }
                         else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                         {
                           this->success = true;
                           state=OPEN_GRIPPER;
                           ros::Duration(0.5).sleep();
                         }
                       }
      break;

      case PILING: ROS_DEBUG("DemosKinovaAlgNode: state PILING");
                   ROS_INFO("Piling.");
                   this->pre_grasp_center.x = tool_pose.x-this->garment_height;
                   this->pre_grasp_center.y = tool_pose.y;
                   this->pre_grasp_center.z = this->garment_height;
                   this->pre_grasp_center.theta_x = 0;
                   this->pre_grasp_center.theta_y = -150;
                   this->pre_grasp_center.theta_z = 180;
                   std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                   this->success &= send_cartesian_pose(this->pre_grasp_center);
                   if (this->success)
                   {
                     ROS_INFO("Success PILING");
                     ros::Duration(0.5).sleep();
                     this->state=PILING2;
                   }
      break;

      case PILING2: ROS_DEBUG("DemosKinovaAlgNode: state PILING2");
                   ROS_INFO("Ending piling");
                   this->pre_grasp_center.x = tool_pose.x-0.1; //minus the width of the already placed garment?? //so the gripper ends at the edge of this garment
                   this->pre_grasp_center.y = tool_pose.y;
                   this->pre_grasp_center.z = 0.1; //Pile height
                   this->pre_grasp_center.theta_x = 0;
                   this->pre_grasp_center.theta_y = -125.5;
                   this->pre_grasp_center.theta_z = 180;
                   std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
                   this->success &= send_cartesian_pose(this->pre_grasp_center);
                   if (this->success)
                   {
                     ROS_INFO("Success PILING2");
                     ros::Duration(0.5).sleep();
                     this->state=OPEN_GRIPPER;
                   }
      break;


      // OPEN GRIPPER
      case OPEN_GRIPPER: ROS_DEBUG("DemosKinovaAlgNode: state OPEN GRIPPER");
                         ROS_INFO("Opening the gripper.");
                         this->success &= send_gripper_command(this->open_gripper);
			 std::cout << "Gripper? " << success << std::endl;
                         if (this->success)
                         {
                           this->state=POST_PLACE;
                           ros::Duration(0.5).sleep();
                         }
			 else
			 {
			   ROS_INFO("DemosKinovaAlgNode: Failed to open gripper");
			   this->state=END;
			 }
      break;

      // POST-PLACE POSITION
      case POST_PLACE: ROS_DEBUG("DemosKinovaAlgNode: state POST PLACE");
                       {
                           ROS_INFO("Sending to post-place position.");
                           geometry_msgs::Pose desired_pose;
                           desired_pose.position.x = tool_pose.x-0.05;
                           desired_pose.position.y = tool_pose.y;
                           desired_pose.position.z = tool_pose.z;
                           std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                           kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                         this->state=WAIT_POST_PLACE;
                       }
      break;

      case WAIT_POST_PLACE: ROS_DEBUG("DemosKinovaAlgNode: state WAIT POST PLACE");
                            {
                              actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                              // to get the state of the current goal
                              this->alg_.unlock();
                              kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                              this->alg_.lock();

                              ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                              if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                              {
                                ROS_INFO("Action aborted!");
                                this->state=END;
                              }
                              else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                              {
                                this->success = true;
                                this->state=HIGH_POSITION;
                                ros::Duration(0.5).sleep();
                              }
                            }
      break;

      // HIGH POSITION
      case HIGH_POSITION: ROS_DEBUG("DemosKinovaAlgNode: state HIGH POSITION");
                          {
                              ROS_INFO("Sending to ending position.");
                              geometry_msgs::Pose desired_pose;
                              desired_pose.position.x = tool_pose.x;
                              desired_pose.position.y = tool_pose.y;
                              desired_pose.position.z = 0.40;
                              std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                              kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                            this->state=WAIT_HIGH_POSITION;
                          }
      break;

      case WAIT_HIGH_POSITION: ROS_DEBUG("DemosKinovaAlgNode: state WAIT HIGH POSITION");
                               {
                                 actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                                 // to get the state of the current goal
                                 this->alg_.unlock();
                                 kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                                 this->alg_.lock();
                                 ROS_DEBUG("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                                 if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                                 {
                                   ROS_INFO("Action aborted!");
                                   this->state=END;
                                 }
                                 else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                                 {
                                   this->success = true;
                                   this->state=END;
                                   ros::Duration(0.5).sleep();
                                 }
                               }
      break;

      case END: ROS_INFO("DemosKinovaAlgNode: state END");
		this->stop=true;
      break;

    }
  }


  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).


  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->cartesian_velocity_publisher_.publish(this->cartesian_velocity_TwistCommand_msg_);

  // Uncomment the following line to publish the topic message
  //this->my_gen3_action_topic_publisher_.publish(this->my_gen3_action_topic_ActionNotification_msg_);
  this->alg_.unlock();
}

// Gets config params
void DemosKinovaAlgNode::get_params(void)
{
  //init class attributes if necessary
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  if(!this->private_node_handle_.getParam("robot_name", this->config_.robot_name))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'robot_name' not found");
  }
  else
    this->robot_name = this->config_.robot_name;

  // Definition parameter pose 00
   if(!this->private_node_handle_.getParam("pre_grasp_corner", this->pre_grasp_corner)) {
       ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'pre_grasp_corner' not found");
   } else {
       ROS_INFO("pre_grasp_corner: [%f, %f, %f, %f, %f, %f]", this->pre_grasp_corner[0], this->pre_grasp_corner[1], this->pre_grasp_corner[2], this->pre_grasp_corner[3], this->pre_grasp_corner[4], this->pre_grasp_corner[5]);
   }
  if(!this->private_node_handle_.getParam("close_gripper", this->config_.close_gripper))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'close_gripper' not found");
  }
  else
    this->close_gripper = this->config_.close_gripper;

  if(!this->private_node_handle_.getParam("garment_width", this->config_.garment_width))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'garment_width' not found");
  }
  else
    this->garment_width = this->config_.garment_width;

  if(!this->private_node_handle_.getParam("garment_height", this->config_.garment_height))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'garment_height' not found");
  }
  else
    this->garment_height = this->config_.garment_height;

  if(!this->private_node_handle_.getParam("diagonal_move", this->config_.diagonal_move))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'diagonal_move' not found");
  }
  else
    this->diagonal_move = this->config_.diagonal_move;
}

// Set and publish handeye transform
void DemosKinovaAlgNode::handeye_frame_pub(const ros::TimerEvent& event)
{

  // Set handeye transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(config_.handeye_x, config_.handeye_y, config_.handeye_z));
  tf::Quaternion q;
  q.setRPY(config_.handeye_r,config_.handeye_p,config_.handeye_yw);
  transform.setRotation(q);
  this->broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","ext_camera_link"));
}

//Subscribes to the topic sending the corner's position
//Renames them according to its position wrt base_link
//Computes edges size
void DemosKinovaAlgNode::corners_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  ROS_DEBUG("DemosKinovaAlgNode: corners callback");

//TO DO:
//Transform corners position wrt base_link
//Get distances to base_link and set corresponding names (down_left, up_right, etc)
//Compute grasp point (another function?)

//Get distances to base_link and set corresponding names (down_left, up_right, etc)
  geometry_msgs::PointStamped point_in;
  geometry_msgs::PointStamped point_out;
  std::vector<geometry_msgs::PointStamped> corners;

  for(int i=0; i<msg->markers.size(); i++)
  {
    point_in.header.frame_id = msg->markers[i].header.frame_id;
    point_in.header.stamp = msg->markers[i].header.stamp;
    point_in.point = msg->markers[i].pose.position;

    this->listener.transformPoint("base_link", point_in, point_out);
    //std::cout << point_in.point.x << std::endl; //Debug
    //std::cout << point_out.point.x << std::endl; //Debug
    corners.push_back(point_out);
  }

  //With point_outs check the two with min x (down) and the two with max x (up)
  //Then from each cluster check the min y (left) and max y (right)



// COMPUTE EDGES
  //geometry_msgs::Point corner_ul, corner_dl, corner_ur, corner_dr; //Better use, as we dont have orientation //points.x
  //visualization_msgs::Marker corner_ul, corner_dl, corner_ur, corner_dr;
  geometry_msgs::Point corner_ul, corner_dl, corner_ur, corner_dr;
  corner_dr = corners[0].point;
  corner_ul = corners[1].point;
  corner_ur = corners[2].point;
  corner_dl = corners[3].point;

  float edge2 = sqrt(pow(abs(corner_ul.x-corner_dl.x),2)+pow(abs(corner_ul.y-corner_dl.y),2));
  float edge1 = sqrt(pow(abs(corner_dr.x-corner_dl.x),2)+pow(abs(corner_dr.y-corner_dl.y),2));

 //DEBUG
/*  std::cout << "\033[1;36m DR -> \033[1;36m  x: " <<  corner_dr.x << " y: " << corner_dr.y << " z: " << corner_dr.z << std::endl;
  std::cout << "\033[1;36m UL -> \033[1;36m  x: " <<  corner_ul.x << " y: " << corner_ul.y << " z: " << corner_ul.z << std::endl;
  std::cout << "\033[1;36m UR -> \033[1;36m  x: " <<  corner_ur.x << " y: " << corner_ur.y << " z: " << corner_ur.z << std::endl;
  std::cout << "\033[1;36m DL -> \033[1;36m  x: " <<  corner_dl.x << " y: " << corner_dl.y << " z: " << corner_dl.z << std::endl;
  std::cout << "\033[1;36m --------------------------" << std::endl;
  //std::cout << "ul-dl: " <<  corner_ul.position.x-corner_dl.position.x << std::endl;
  //std::cout << "dr-dl: " <<  edge1 << std::endl;
  //std::cout << "\033[1;36m --------------------------" << std::endl;
  std::cout << "Edge1: " <<  edge1 << std::endl;
  std::cout << "Edge2: " <<  edge2 << std::endl;
  std::cout << "\033[1;36m --------------------------" << std::endl;
  std::cout << "\033[1;36m --------------------------" << std::endl; */

// COMPUTES GRASP POINT - In another function?
  geometry_msgs::Point grasp_point;
  double u1, u2; //Edge direction vector components
  std_msgs::Float64 grasp_angle;
  float cos_alpha, alpha;
  std_msgs::Float64 garment_edge;

  // Check if the object's shape is a square or a rectangle
  if(abs(edge1-edge2)>0.05)
  {
    // Get mid point of longest edge
    if(edge1 > edge2)
    {
      //std::cout << "Edge1!" << std::endl; //Debug
      grasp_point.x = corner_dl.x + (corner_dr.x - corner_dl.x)/2;
      grasp_point.y = corner_dl.y - (abs(corner_dl.y - corner_dr.y)/2); //REVISAR?
      grasp_point.z = corner_dl.z+0.02;
      garment_edge.data = abs(edge2);
      u1 = corner_dl.x - corner_dr.x;    //X direction of edge vector
      u2 = corner_dl.y - corner_dr.y;    //Y direction of edge vector
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
        if(u1>0) //corner_dl.x > corner_dr.x = Diagonal izq
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
      grasp_point.x = corner_dl.x + (abs(corner_dl.x - corner_ul.x)/2); //REVISAR!
      grasp_point.y = corner_dl.y + (corner_ul.y - corner_dl.y)/2; //REVISAR?
      grasp_point.z = corner_dl.z+0.02;
      garment_edge.data = abs(edge1);
      u1 = corner_ul.x - corner_dl.x;    //X direction of edge vector
      u2 = corner_ul.y - corner_dl.y;    //Y direction of edge vector
      cos_alpha = (abs(u2))/(sqrt(pow(u1,2)+pow(u2,2)));
      alpha = acos(cos_alpha);
      if(alpha > 1)
      {
        grasp_point.x = corner_dr.x + (abs(corner_dr.x - corner_ur.x)/2); //REVISAR!
        grasp_point.y = corner_dr.y + (corner_ur.y - corner_dr.y)/2; //REVISAR?
        grasp_point.z = corner_dr.z+0.02;
        grasp_angle.data = 3;
        //std::cout << "VERTICAL: E2 > 1" << std::endl; //Debug
      }
      if(1 > alpha > 0.3)
      {
        grasp_point.x = corner_dr.x + (abs(corner_dr.x - corner_ur.x)/2); //REVISAR!
        grasp_point.y = corner_dr.y + (corner_ur.y - corner_dr.y)/2; //REVISAR?
        grasp_point.z = corner_dr.z+0.02;
        //std::cout << "DIAGONAL DER: E2 > 0.3-1.2" << std::endl; //Debug
        grasp_angle.data = 2;
      }
    }
  }
  else //Squared object -> Get mid point of closet edge
  {
    //std::cout << "Square!" << std::endl; //Debug
    grasp_point.x = corner_dl.x + (corner_dr.x - corner_dl.x)/2;
    grasp_point.y = corner_dl.y - (abs(corner_dl.y - corner_dr.y)/2);
    grasp_point.z = corner_dl.z+0.02;
    u1 = corner_dl.x - corner_dr.x;    //X direction of edge vector
    u2 = corner_dl.y - corner_dr.y;    //Y direction of edge vector
    garment_edge.data = abs(edge2);
  }

  //std::cout << "\033[1;36m GRASP POINT -> \033[1;36m  x: " <<  grasp_point.x << " y: " << grasp_point.y << " z: " << grasp_point.z << std::endl;

  //Publish grasp point marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x=0.01;
  marker.scale.y=0.01;
  marker.scale.z=0.01;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  marker.pose.position.x=grasp_point.x;
  marker.pose.position.y=grasp_point.y;
  marker.pose.position.z=grasp_point.z;
  garment_marker_publisher.publish(marker);


  if(this->get_garment_position)
  {
    this->compute_grasp_angle(grasp_angle);
    this->pre_grasp_center.x = grasp_point.x-this->pre_grasp_distance.x;
    this->pre_grasp_center.y = grasp_point.y-this->pre_grasp_distance.y;
    this->pre_grasp_center.z = 0.055;
    this->grasping_point_garment = this->pre_grasp_center;

    std::cout << "\033[1;36m GRASP POINT -> \033[1;36m  x: " <<  pre_grasp_center.x << " y: " << pre_grasp_center.y << " z: " << pre_grasp_center.z << std::endl;
    marker.pose.position.x=pre_grasp_center.x;
    marker.pose.position.y=pre_grasp_center.y;
    marker.pose.position.z=pre_grasp_center.z;
    grasp_marker_publisher.publish(marker);

    this->get_garment_position=false;
  }

}

// Transform point obtained through camera wrt robot frame base_link
void DemosKinovaAlgNode::garment_pose_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  ROS_DEBUG("DemosKinovaAlgNode: garment pose callback");
/*
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x=0.01;
  marker.scale.y=0.01;
  marker.scale.z=0.01;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  geometry_msgs::PointStamped point_in;
  geometry_msgs::PointStamped point_out;

  point_in.header.frame_id = msg->header.frame_id;
  point_in.header.stamp = msg->header.stamp;
  point_in.point = msg->pose.position;

  if(this->get_garment_position)
  {
    this->listener.transformPoint("base_link", point_in, point_out);
//    this->grasp_pose.pose.position = point_out.point;
//    this->grasp_pose.pose.orientation =
//    std::cout << "\033[1;36m Grasp position -> \033[1;36m  x: " << grasp_pose.pose.position.x << ", y: " << grasp_pose.pose.position.y << ", z: " << grasp_pose.pose.position.z << std::endl;

    this->pre_grasp_center.x = point_out.point.x-this->pre_grasp_distance.x;
    this->pre_grasp_center.y = point_out.point.y-this->pre_grasp_distance.y;
    this->pre_grasp_center.z = 0.055;

    std::cout << "Garment position -> x: " << point_out.point.x << ", y: " << point_out.point.y << ", z: " << point_out.point.z << std::endl;

    std::cout << "\033[1;36m Grasp position -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
    std::cout << "\033[1;36m Grasp orientation -> \033[1;36m  x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;
    this->grasping_point_garment = this->pre_grasp_center;

    marker.pose.position.x=point_out.point.x;
    marker.pose.position.y=point_out.point.y;
    marker.pose.position.z=point_out.point.z;
//    marker.pose.position.x=this->pre_grasp_center.x;
//    marker.pose.position.y=this->pre_grasp_center.y;
//    marker.pose.position.z=this->pre_grasp_center.z;
    grasp_marker_publisher.publish(marker);

    this->get_garment_position=false;
  }*/
}


//void DemosKinovaAlgNode::garment_angle_callback(const std_msgs::Float64::ConstPtr& msg)
void DemosKinovaAlgNode::compute_grasp_angle(const std_msgs::Float64& garment_angle)
{
  ROS_DEBUG("DemosKinovaAlgNode: garment angle callback");

//  if(this->get_garment_angle)
//  {
  if(garment_angle.data == 0)
  {
    std::cout << "\033[1;36m Gripper orientation HORIZONTAL" <<  std::endl;
    this->pre_grasp_distance.x = 0.06;
    this->pre_grasp_distance.y = 0;
    this->pre_grasp_center.theta_x = 0;
    this->pre_grasp_center.theta_y = -125.5;
    this->pre_grasp_center.theta_z = 180;
  }
  else if(garment_angle.data == 1)
  {
    std::cout << "\033[1;36m Gripper horientation DIAG IZQUIERDA" <<  std::endl;
    this->pre_grasp_distance.x = 0.08;
    this->pre_grasp_distance.y = -0.08;
    this->pre_grasp_center.theta_x = 0;
    this->pre_grasp_center.theta_y = -120;
    this->pre_grasp_center.theta_z = 135;
  }
  else if(garment_angle.data == 2)
  {
    std::cout << "\033[1;36m Gripper horientation DIAG DERECHA" <<  std::endl;
    this->pre_grasp_distance.x = 0.08;
    this->pre_grasp_distance.y = 0.08;
    this->pre_grasp_center.theta_x = -176;
    this->pre_grasp_center.theta_y = -52;
    this->pre_grasp_center.theta_z = 37;
  }
  else if(garment_angle.data == 3)
  {
    std::cout << "\033[1;36m Gripper horientation VERTICAL" << std::endl;
    this->pre_grasp_distance.x = 0;
    this->pre_grasp_distance.y = 0.04;
    this->pre_grasp_center.theta_x = -179; //-170;
    this->pre_grasp_center.theta_y = -50; //-52;
    this->pre_grasp_center.theta_z = 85; //55;
  }
  this->get_garment_angle=false;
  this->get_garment_position=true;

  std::cout << "Defined orientation: ->   x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;
  std::cout << "Distances: ->   x: " << this->pre_grasp_distance.x << ", y: " << this-> pre_grasp_distance.y << ", z: " << std::endl;
  //}
}

void DemosKinovaAlgNode::garment_edge_callback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_DEBUG("DemosKinovaAlgNode: garment edge callback");
  if(this->get_garment_position)
  {
    this->garment_height=msg->data +0.01;
    std::cout << "Non-grasped edge size: " << this->garment_height << std::endl;
    if(this->garment_height>0.26)
      this->garment_height=0.26;
//      grasp point 0.4
  }
}

/*  [subscriber callbacks] */
void DemosKinovaAlgNode::base_feedback_callback(const kortex_driver::BaseCyclic_Feedback::ConstPtr& msg)
{
  //ROS_INFO("DemosKinovaAlgNode::base_feedback_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->base_feedback_mutex_enter();
  tool_pose.x = msg->base.tool_pose_x;
  tool_pose.y = msg->base.tool_pose_y;
  tool_pose.z = msg->base.tool_pose_z;
  tool_pose.theta_x = msg->base.tool_pose_theta_x;
  tool_pose.theta_y = msg->base.tool_pose_theta_y;
  tool_pose.theta_z = msg->base.tool_pose_theta_z;

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->base_feedback_mutex_exit();
}

void DemosKinovaAlgNode::base_feedback_mutex_enter(void)
{
  pthread_mutex_lock(&this->base_feedback_mutex_);
}

void DemosKinovaAlgNode::base_feedback_mutex_exit(void)
{
  pthread_mutex_unlock(&this->base_feedback_mutex_);
}

void DemosKinovaAlgNode::action_topic_callback(const kortex_driver::ActionNotification::ConstPtr& msg)
{
//  ROS_INFO("DemosKinovaAlgNode::action_topic_callback: New Message Received");
  this->last_action_notification_event = msg->action_event;
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->action_topic_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->action_topic_mutex_exit();
}

void DemosKinovaAlgNode::action_topic_mutex_enter(void)
{
  pthread_mutex_lock(&this->action_topic_mutex_);
}

void DemosKinovaAlgNode::action_topic_mutex_exit(void)
{
  pthread_mutex_unlock(&this->action_topic_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */
void DemosKinovaAlgNode::kinova_linear_moveDone(const actionlib::SimpleClientGoalState& state,  const iri_kinova_linear_movement::kinova_linear_movementResultConstPtr& result)
{
  this->alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveDone: Goal Achieved!");
  else
    ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveDone: %s", state.toString().c_str());

  //copy & work with requested result
  this->alg_.unlock();
}

void DemosKinovaAlgNode::kinova_linear_moveActive()
{
  this->alg_.lock();
  //ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveActive: Goal just went active!");
  this->alg_.unlock();
}

void DemosKinovaAlgNode::kinova_linear_moveFeedback(const iri_kinova_linear_movement::kinova_linear_movementFeedbackConstPtr& feedback)
{
  this->alg_.lock();
  //ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    kinova_linear_move_client_.cancelGoal();
    //ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveFeedback: Cancelling Action!");
  }
  this->alg_.unlock();
}


/*  [action requests] */
bool DemosKinovaAlgNode::kinova_linear_moveMakeActionRequest(const geometry_msgs::Pose& desired_pose, const int& rf_frame, const float& max_vel)
{
  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  bool ok;
  // this->alg_.unlock();
  if(kinova_linear_move_client_.isServerConnected())
  {
    //ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //kinova_linear_move_goal_.data = my_desired_goal;
    kinova_linear_move_goal_.goal_position = desired_pose;
    kinova_linear_move_goal_.reference_frame = rf_frame;
    kinova_linear_move_goal_.maximum_velocity = max_vel;
    kinova_linear_move_client_.sendGoal(kinova_linear_move_goal_,
                boost::bind(&DemosKinovaAlgNode::kinova_linear_moveDone,     this, _1, _2),
                boost::bind(&DemosKinovaAlgNode::kinova_linear_moveActive,   this),
                boost::bind(&DemosKinovaAlgNode::kinova_linear_moveFeedback, this, _1));
    ROS_INFO("DemosKinovaAlgNode::kinova_linear_moveMakeActionRequest: Goal Sent.");
    // ok=true;
    return true;
  }
  else
  {
    ROS_ERROR("DemosKinovaAlgNode::kinova_linear_moveMakeActionRequest: action server is not connected. Check remap or server presence.");
    // ok=false;
    return false;
  }
    // this->alg_.lock();
    return ok;
}


void DemosKinovaAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  if(config.rate!=this->getRate())
    this->setRate(config.rate);
  if(config.diagonal_move != this->diagonal_move)
  {
    this->diagonal_move = config.diagonal_move;
    std::cout << this->diagonal_move << std::endl;
  }
  if(config.start)
  {
    this->start=true;
    this->close_gripper=config.close_gripper;
    config.start=false;
  }
  if(config.stop)
    this->stop=true;
  if(config.get_grasp_point)
  {
    this->get_garment_position=true;
//    this->get_garment_angle=true;
    config.get_grasp_point=false;
  }
  if(config.test)
  {
    this->pre_grasp_center.x = config.grasp_x;
    this->pre_grasp_center.y = config.grasp_y;
    this->pre_grasp_center.z = config.grasp_z;
    this->pre_grasp_center.theta_x = config.grasp_thetax;
    this->pre_grasp_center.theta_y = config.grasp_thetay;
    this->pre_grasp_center.theta_z = config.grasp_thetaz;
    std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
    std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;
    config.test=false;
  }

  this->config_=config;
  this->alg_.unlock();
}

void DemosKinovaAlgNode::addNodeDiagnostics(void)
{
}

bool DemosKinovaAlgNode::clear_faults(void)
{
  // Clear the faults
  if (base_clear_faults_client_.call(base_clear_faults_srv_))
  {
  //  ROS_INFO("DemosKinovaAlgNode:: base_clear_faults_client_ received a response from service server");
    ROS_INFO("DemosKinovaAlgNode:: Clear the faults");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->base_clear_faults_client_.getService().c_str());
    return false;
  }
  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

// This function sets the reference frame to the robot's base
bool DemosKinovaAlgNode::set_cartesian_reference_frame(const int &cartesian_rf)
{
  set_cartesian_rf_srv_.request.input.reference_frame = cartesian_rf;
  this->cartesian_rf = cartesian_rf;
  ROS_INFO("DemosKinovaAlgNode:: Calling service set_cartesian_rf_client_!");
  if (set_cartesian_rf_client_.call(set_cartesian_rf_srv_))
  {
    ROS_INFO("Setting reference frame.");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->set_cartesian_rf_client_.getService().c_str());
    return false;
  }
  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  return true;
}

bool DemosKinovaAlgNode::send_gripper_command(double value)
{
  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  send_gripper_cmd_srv_.request.input.gripper.finger.push_back(finger);
  send_gripper_cmd_srv_.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;
  ROS_INFO("DemosKinovaAlgNode:: Calling service send_gripper_cmd_client_!");
  if (send_gripper_cmd_client_.call(send_gripper_cmd_srv_))
  {
    ROS_INFO("The gripper command was sent to the robot.");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->send_gripper_cmd_client_.getService().c_str());
    return false;
  }
  ROS_INFO("DemosKinovaAlgNode: Gripper command sended");
  send_gripper_cmd_srv_.request.input.gripper.finger.pop_back();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool DemosKinovaAlgNode::home_the_robot(void)
{
  this->last_action_notification_event = 0;
  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  base_read_action_srv_.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!base_read_action_client_.call(base_read_action_srv_))
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->base_read_action_client_.getService().c_str());
    return false;
  }

  // We can now execute the Action that we read
  base_execute_action_srv_.request.input = base_read_action_srv_.response.output;
  ROS_INFO("DemosKinovaAlgNode:: Calling service base_execute_action_client_!");
  if (base_execute_action_client_.call(base_execute_action_srv_))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->base_execute_action_client_.getService().c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  return wait_for_action_end_or_abort();
}

bool DemosKinovaAlgNode::send_cartesian_pose(const kortex_driver::Pose &goal_pose)
{
  ROS_DEBUG("DemosKinovaAlgNode: send_cartesian_pose function");
  this->last_action_notification_event = 0;
  kortex_driver::Waypoint waypoint;
  exec_wp_trajectory_srv_.request.input.waypoints.clear();
  exec_wp_trajectory_srv_.request.input.waypoints.push_back(FillCartesianWaypoint(goal_pose, 0));
  exec_wp_trajectory_srv_.request.input.duration = 0;
  exec_wp_trajectory_srv_.request.input.use_optimal_blending = false;

  ROS_INFO("DemosKinovaAlgNode:: Calling service exec_wp_trajectory_client_!");
  if (exec_wp_trajectory_client_.call(exec_wp_trajectory_srv_))
  {
    ROS_INFO("The new cartesian pose was sent to the robot.");
  }
  else
  {
    ROS_INFO("DemosKinovaAlgNode:: Failed to call service on topic %s",this->exec_wp_trajectory_client_.getService().c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return wait_for_action_end_or_abort();
}

kortex_driver::Waypoint DemosKinovaAlgNode::FillCartesianWaypoint(const kortex_driver::Pose &goal_pose, float blending_radius)
{
  kortex_driver::Waypoint waypoint;
  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = goal_pose.x;
  cartesianWaypoint.pose.y = goal_pose.y;
  cartesianWaypoint.pose.z = goal_pose.z;
  cartesianWaypoint.pose.theta_x = goal_pose.theta_x;
  cartesianWaypoint.pose.theta_y = goal_pose.theta_y;
  cartesianWaypoint.pose.theta_z = goal_pose.theta_z;
  cartesianWaypoint.reference_frame =  kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = blending_radius;

  waypoint.oneof_type_of_waypoint.cartesian_waypoint.push_back(cartesianWaypoint);

  return waypoint;
}

//REVISAR!! Quitar while, poner comprobacion dentro de sm, mirar gestion estado actions tiago modules
bool DemosKinovaAlgNode::wait_for_action_end_or_abort(void)
{
  ROS_DEBUG("DemosKinovaAlgNode: wait_for_action_end_or_abort");
  while (ros::ok())
  {
    if (this->last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification");
      return true;
    }
    else if (this->last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    else
      ROS_DEBUG("Wait for action end or abort"); //print state

    ros::spinOnce();
  }
  return false;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DemosKinovaAlgNode>(argc, argv, "demos_kinova_alg_node");
}
