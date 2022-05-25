#include "demos_kinova_alg_node.h"

DemosKinovaAlgNode::DemosKinovaAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DemosKinovaAlgorithm>(),
  kinova_linear_move_client_(private_node_handle_,"kinova_linear_move", true)
{

  this->start=false;
  this->stop=false;

  // Garment pose subscriber
  this->garment_pose_subscriber = this->public_node_handle_.subscribe("/segment_table/grasp_point",1,&DemosKinovaAlgNode::garment_pose_callback,this);
  this->get_garment_pose=false;

  // Publish grasp marker
  this->grasp_marker_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("grasp_marker", 1);

  // Publish external camera frame 
  this->handeye_frame_pub_timer = this->public_node_handle_.createTimer(ros::Duration(1.0),&DemosKinovaAlgNode::handeye_frame_pub,this);
  this->handeye_frame_pub_timer.stop();

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

  double open_gripper = 0.35;
  double close_gipper = 0.97; //0.81;

  if(this->stop)
  {
    ROS_WARN("Demo Pick n Place has stopped!");
    state=0;
    this->stop=false;
  }
  // OPEN GRIPPER
  if (state == 0)
  {
    if(this->start)
    {
      ROS_INFO("Opening the gripper.");
      this->success &= send_gripper_command(open_gripper);
      if (this->success)
      {
        state++;
        ros::Duration(0.5).sleep();
        this->start=false;
      }
    } 
  }
  // HOME POSITION
  if (state == 1)
  {
    // Move the robot to the Home position with an Action
    ROS_INFO("Moving to home position.");
    this->success &= home_the_robot();
    if (this->success)
    {
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // PRE-GRASP POSITION
  if (state == 2)
  {
    ROS_INFO("Sending to pre-grasp position.");
    std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
    this->success &= send_cartesian_pose(this->pre_grasp_center);
    if (this->success)
    {
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // GRASP POSITION
  if (state == 3)
  {
    // [fill action structure and make request to the action server]
    // variable to hold the state of the current goal on the server
    actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
    static bool first=true;
    if(first){
      ROS_INFO("Sending to grasp position.");
      first=false;
      geometry_msgs::Pose desired_pose;
      desired_pose.position.x = tool_pose.x + 0.04;
      desired_pose.position.y = tool_pose.y;
      desired_pose.position.z = tool_pose.z;
      std::cout << "\033[1;36m Groing to: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.1);
    }
    // to get the state of the current goal
    this->alg_.unlock();
    kinova_linear_move_state=kinova_linear_move_client_.getState();
    // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
    this->alg_.lock();
    ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
    if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("Action aborted!");
    }
    else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->success = true;
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // CLOSE GRIPPER
  if (state == 4)
  {
    ROS_INFO("Closing the gripper.");
    this->success &= send_gripper_command(close_gipper);
    if (this->success)
    {
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // POST-GRASP POSITION
  if (state == 5)
  {
    // [fill action structure and make request to the action server]
    // variable to hold the state of the current goal on the server
    actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
    static bool first=true;
    if(first){
      ROS_INFO("Sending to post-grasp position.");
      first=false;
      geometry_msgs::Pose desired_pose;
      desired_pose.position.x = tool_pose.x;
      desired_pose.position.y = tool_pose.y;
      desired_pose.position.z = tool_pose.z + this->garment_height*1.5;
      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
    }
    // to get the state of the current goal
    this->alg_.unlock();
    kinova_linear_move_state=kinova_linear_move_client_.getState();
    // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
    this->alg_.lock();
    ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
    if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("Action aborted!");
    }
    else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->success = true;
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // PRE-PLACE POSITION
  if (state == 6)
  {
    // [fill action structure and make request to the action server]
    // variable to hold the state of the current goal on the server
    actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
    static bool first=true;
    if(first){
      ROS_INFO("Sending to post-grasp position.");
      first=false;
      geometry_msgs::Pose desired_pose;
      desired_pose.position.x = 0.7;
      desired_pose.position.y = -this->garment_width/2;
      desired_pose.position.z = tool_pose.z;
      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
    }
    // to get the state of the current goal
    this->alg_.unlock();
    kinova_linear_move_state=kinova_linear_move_client_.getState();
    // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
    this->alg_.lock();
    ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
    if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("Action aborted!");
    }
    else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->success = true;
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  if (this->diagonal_move)
  {
    if (state == 7)
    {
      // [fill action structure and make request to the action server]
      // variable to hold the state of the current goal on the server
      actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
      static bool first=true;
      if(first){
        ROS_INFO("Sending to diagonal position.");
        first=false;
        geometry_msgs::Pose desired_pose;
        desired_pose.position.x = tool_pose.x;
        desired_pose.position.y =  tool_pose.y;
        desired_pose.position.z = this->garment_height;
        kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
      }
      // to get the state of the current goal
      this->alg_.unlock();
      kinova_linear_move_state=kinova_linear_move_client_.getState();
      // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
      this->alg_.lock();
      ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
      if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_INFO("Action aborted!");
      }
      else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        this->success = true;
        state++;
        ros::Duration(0.5).sleep();
      }
    }
    // PLACE POSITION
    if (state == 8)
    {
      // [fill action structure and make request to the action server]
      // variable to hold the state of the current goal on the server
      actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
      static bool first=true;
      if(first){
        ROS_INFO("Sending to place position.");
        first=false;
        geometry_msgs::Pose desired_pose;
        desired_pose.position.x = tool_pose.x-this->garment_height/1.5;
        desired_pose.position.y = tool_pose.y;
        desired_pose.position.z = this->pre_grasp_center.z;
        kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
      }
      // to get the state of the current goal
      this->alg_.unlock();
      kinova_linear_move_state=kinova_linear_move_client_.getState();
      // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
      this->alg_.lock();
      ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
      if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_INFO("Action aborted!");
      }
      else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        this->success = true;
        state++;
        ros::Duration(0.5).sleep();
      }
    }
  }
  else
  {
    // PLACE POSITION
    if (state == 7)
    {
      // [fill action structure and make request to the action server]
      // variable to hold the state of the current goal on the server
      actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
      static bool first=true;
      if(first){
        ROS_INFO("Sending to place position.");
        first=false;
        geometry_msgs::Pose desired_pose;
        desired_pose.position.x = tool_pose.x;
        desired_pose.position.y = tool_pose.y;
        desired_pose.position.z = 0.11;
        kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
      }
      // to get the state of the current goal
      this->alg_.unlock();
      kinova_linear_move_state=kinova_linear_move_client_.getState();
      // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
      this->alg_.lock();
      ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
      if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_INFO("Action aborted!");
      }
      else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        this->success = true;
        state += 2;
        ros::Duration(0.5).sleep();
      }
    }
  }
  // OPEN GRIPPER
  if (state == 9)
  {
    ROS_INFO("Opening the gripper.");
    this->success &= send_gripper_command(open_gripper);
    if (this->success)
    {
      state++;
      ros::Duration(0.5).sleep();
    }
  }
  // POST-PLACE POSITION
  if (state == 10)
  {
    // [fill action structure and make request to the action server]
    // variable to hold the state of the current goal on the server
    actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
    static bool first=true;
    if(first){
      ROS_INFO("Sending to place position.");
      first=false;
      geometry_msgs::Pose desired_pose;
      desired_pose.position.x = tool_pose.x-0.05;
      desired_pose.position.y = tool_pose.y;
      desired_pose.position.z = tool_pose.z;
      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
    }
    // to get the state of the current goal
    this->alg_.unlock();
    kinova_linear_move_state=kinova_linear_move_client_.getState();
    // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
    this->alg_.lock();
    ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
    if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("Action aborted!");
    }
    else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->success = true;
      state++;
      ros::Duration(0.5).sleep();
    }
  }


  // HIGH POSITION
  if (state == 11)
  {
    // [fill action structure and make request to the action server]
    // variable to hold the state of the current goal on the server
    actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
    static bool first=true;
    if(first){
      ROS_INFO("Sending to place position.");
      first=false;
      geometry_msgs::Pose desired_pose;
      desired_pose.position.x = tool_pose.x;
      desired_pose.position.y = tool_pose.y;
      desired_pose.position.z = 0.50;
      kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
    }
    // to get the state of the current goal
    this->alg_.unlock();
    kinova_linear_move_state=kinova_linear_move_client_.getState();
    // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
    this->alg_.lock();
    ROS_INFO("DemosKinovaAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
    if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("Action aborted!");
    }
    else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      this->success = true;
      state++;
      ros::Duration(0.5).sleep();
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

// Starts grasp point marker and grasping frame topics with given rates 
void DemosKinovaAlgNode::set_config(void)
{
  ROS_INFO("DemosKinovaAlgNode: Set configuration");

  // Start publisher rates 
  this->handeye_frame_pub_timer.setPeriod(ros::Duration(1.0/50.0));
  this->handeye_frame_pub_timer.start();
}

// Create and publish finger tip and garment's corner frames 
void DemosKinovaAlgNode::handeye_frame_pub(const ros::TimerEvent& event)
{
  
  // Create finger tip frame 
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(config_.x, config_.y, config_.z));
  tf::Quaternion q;
  q.setRPY(config_.r,config_.p,config_.yaw);
  transform.setRotation(q); 
  this->broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","ext_camera_link"));
}

// Transform point obtained through camera wrt robot frame base_link
void DemosKinovaAlgNode::garment_pose_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  ROS_DEBUG("DemosKinovaAlgNode: garment pose callback");

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

  if(this->get_garment_pose)
  {
    this->listener.transformPoint("base_link", point_in, point_out);
//    this->grasp_pose.pose.position = point_out.point;
//    this->grasp_pose.pose.orientation = 
//    std::cout << "\033[1;36m Grasp position -> \033[1;36m  x: " << grasp_pose.pose.position.x << ", y: " << grasp_pose.pose.position.y << ", z: " << grasp_pose.pose.position.z << std::endl;

    this->pre_grasp_center.x = point_out.point.x-0.09;
    this->pre_grasp_center.y = point_out.point.y;
    std::cout << "\033[1;36m Grasp position -> \033[1;36m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
    std::cout << "\033[1;36m Grasp orientation -> \033[1;36m  x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;

    marker.pose.position.x=point_out.point.x;
    marker.pose.position.y=point_out.point.y;
    marker.pose.position.z=point_out.point.z;
    grasp_marker_publisher.publish(marker);

    this->get_garment_pose=false;
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
    this->start=true;
  if(config.stop)
    this->stop=true;
  if(config.set_config)
    this->set_config();
  if(config.test)
    this->get_garment_pose=true;

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
  this->last_action_notification_event = 0;
  kortex_driver::Waypoint waypoint;
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

bool DemosKinovaAlgNode::wait_for_action_end_or_abort(void)
{
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
    ros::spinOnce();
  }
  return false;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DemosKinovaAlgNode>(argc, argv, "demos_kinova_alg_node");
}
