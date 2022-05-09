#include "kinova_linear_movement_alg_node.h"

KinovaLinearMovementAlgNode::KinovaLinearMovementAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<KinovaLinearMovementAlgorithm>(),
  kinova_linear_move_aserver_(private_node_handle_, "kinova_linear_move")
{
  //init class attributes if necessary
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("KinovaLinearMovementAlgNode::KinovaLinearMovementAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  if(!this->private_node_handle_.getParam("robot_name", this->config_.robot_name))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'robot_name' not found");
  }
  else
    this->robot_name = this->config_.robot_name;

  if(!this->private_node_handle_.getParam("action_time_limit", this->config_.action_time_limit))
  {
    ROS_WARN("DemosKinovaAlgNode::DemosKinovaAlgNode: param 'action_time_limit' not found");
  }
  else
    this->action_time_limit = this->config_.action_time_limit;
  // [init publishers]
  this->cartesian_velocity_publisher_ = this->private_node_handle_.advertise<kortex_driver::TwistCommand>("/" + this->robot_name + "/in/cartesian_velocity", 1);

  // [init subscribers]
  this->base_feedback_subscriber_ = this->private_node_handle_.subscribe("/" + this->robot_name  + "/base_feedback", 1000, &KinovaLinearMovementAlgNode::base_feedback_callback, this);
  pthread_mutex_init(&this->base_feedback_mutex_,NULL);


  // [init services]

  // [init clients]

  // [init action servers]
  kinova_linear_move_aserver_.registerStartCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveStartCallback, this, _1));
  kinova_linear_move_aserver_.registerStopCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveStopCallback, this));
  kinova_linear_move_aserver_.registerIsFinishedCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveIsFinishedCallback, this));
  kinova_linear_move_aserver_.registerHasSucceedCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveHasSucceededCallback, this));
  kinova_linear_move_aserver_.registerGetResultCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveGetResultCallback, this, _1));
  kinova_linear_move_aserver_.registerGetFeedbackCallback(boost::bind(&KinovaLinearMovementAlgNode::kinova_linear_moveGetFeedbackCallback, this, _1));
  kinova_linear_move_aserver_.start();
  this->kinova_linear_move_active=false;
  this->kinova_linear_move_succeeded=false;
  this->kinova_linear_move_finished=false;


  // [init action clients]
}

KinovaLinearMovementAlgNode::~KinovaLinearMovementAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->base_feedback_mutex_);
}

void KinovaLinearMovementAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("KinovaLinearMovementAlgNode::mainNodeThread");
  // [fill msg structures]
  // Initialize the topic message structure
  //this->cartesian_velocity_TwistCommand_msg_.data = my_var;


  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]
  // To finish the action server with success
  //this->kinova_linear_move_succeeded=true;
  //this->kinova_linear_move_finished=true;
  // To finish the action server with failure
  //this->kinova_linear_move_succeeded=false;
  //this->kinova_linear_move_finished=true;

  // IMPORTANT: it is better to use the boolean variables to control the
  // behavior of the action server instead of direclty calling the action server
  // class functions.


  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->cartesian_velocity_publisher_.publish(this->cartesian_velocity_TwistCommand_msg_);
  if (this->kinova_linear_move_active && !this->kinova_linear_move_finished && !this->kinova_linear_move_succeeded) {
    kortex_driver::TwistCommand stop_msg;
    tf::Vector3 current_pos(tool_pose.x, tool_pose.y, tool_pose.z);
    tf::Vector3 error = this->goal_pos - current_pos;
    tf::Vector3 ac_error(0.0, 0.0, 0.0);
    tf::Vector3 threshold (0.0015, 0.0015, 0.0015);
    this->twist_cmd = get_velocity(error);
    this->cartesian_velocity_publisher_.publish(this->twist_cmd); // Publish the speed that is applied to the robot
    if ((this->goal_pos.x() - threshold.x()) < current_pos.x() && current_pos.x() < (this->goal_pos.x() + threshold.x()) &&
        (this->goal_pos.y() - threshold.y()) < current_pos.y() && current_pos.y() < (this->goal_pos.y() + threshold.y()) &&
        (this->goal_pos.z() - threshold.z()) < current_pos.z() && current_pos.z() < (this->goal_pos.z() + threshold.z()))
    {
      this->cartesian_velocity_publisher_.publish(stop_msg); // Publish the speed that is applied to the robot
      this->kinova_linear_move_succeeded=true;
      this->kinova_linear_move_finished=true;
    }
    this->total_time = ros::Time::now() - this->start_time;
    if (total_time >= ros::Duration(this->action_time_limit)){
      this->cartesian_velocity_publisher_.publish(stop_msg); // Publish the speed that is applied to the robot
      this->kinova_linear_move_succeeded=false;
      this->kinova_linear_move_finished=true;
    }
  }

  if (this->stop_robot)
  {
    kortex_driver::TwistCommand stop_msg;
    this->cartesian_velocity_publisher_.publish(stop_msg);
    this->stop_robot = false;
  }
  /*if (this->kinova_linear_move_finished || this->kinova_linear_move_succeeded)
  {
    kortex_driver::TwistCommand stop_msg;
    this->cartesian_velocity_publisher_.publish(stop_msg); // Publish the speed that is applied to the robot
  }*/
  this->alg_.unlock();
}

/*  [subscriber callbacks] */
void KinovaLinearMovementAlgNode::base_feedback_callback(const kortex_driver::BaseCyclic_Feedback::ConstPtr& msg)
{
  //ROS_INFO("KinovaLinearMovementAlgNode::base_feedback_callback: New Message Received");

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

void KinovaLinearMovementAlgNode::base_feedback_mutex_enter(void)
{
  pthread_mutex_lock(&this->base_feedback_mutex_);
}

void KinovaLinearMovementAlgNode::base_feedback_mutex_exit(void)
{
  pthread_mutex_unlock(&this->base_feedback_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */
void KinovaLinearMovementAlgNode::kinova_linear_moveStartCallback(const iri_kinova_linear_movement::kinova_linear_movementGoalConstPtr& goal)
{
  ROS_INFO_STREAM("I GOT A NEW GOAL!");
  ROS_INFO_STREAM("X pose: " << goal->goal_position.position.x);
  ROS_INFO_STREAM("Y pose: " << goal->goal_position.position.y);
  ROS_INFO_STREAM("Z pose: " << goal->goal_position.position.z);
  ROS_INFO_STREAM("reference frame: " << goal->reference_frame);
  ROS_INFO_STREAM("maximum_velocity: " << goal->maximum_velocity);
  //this->alg_.lock();
  //check goal
  this->kinova_linear_move_active=true;
  this->kinova_linear_move_succeeded=false;
  this->kinova_linear_move_finished=false;
  //execute goal
  this->goal_pos = tf::Vector3(goal->goal_position.position.x, goal->goal_position.position.y, goal->goal_position.position.z);
  this->twist_cmd.reference_frame = goal->reference_frame;
  this->twist_cmd.duration = 0;
  this->maximum_vel = goal->maximum_velocity;
  this->total_time = ros::Duration(0);
  this->start_time = ros::Time::now();

  //this->alg_.unlock();
}

void KinovaLinearMovementAlgNode::kinova_linear_moveStopCallback(void)
{
  //this->alg_.lock();
  //stop action
  this->kinova_linear_move_active=false;
  this->stop_robot = true;
  ROS_INFO_STREAM("STOPPED ACTION!");
  //this->alg_.unlock();
}

bool KinovaLinearMovementAlgNode::kinova_linear_moveIsFinishedCallback(void)
{
  bool ret = false;

  //this->alg_.lock();
  //if action has finish for any reason
  ret = this->kinova_linear_move_finished;
  //this->alg_.unlock();

  return ret;
}

bool KinovaLinearMovementAlgNode::kinova_linear_moveHasSucceededCallback(void)
{
  bool ret = false;

  //this->alg_.lock();
  //if goal was accomplished
  ret = this->kinova_linear_move_succeeded;
  this->kinova_linear_move_active=false;
  ROS_INFO_STREAM("SUCCEEDED ACTION!");
  //this->alg_.unlock();

  return ret;
}

void KinovaLinearMovementAlgNode::kinova_linear_moveGetResultCallback(iri_kinova_linear_movement::kinova_linear_movementResultPtr& result)
{
  //this->alg_.lock();
  //update result data to be sent to client
  //result->data = data;
  result->final_position.position.x = tool_pose.x;
  result->final_position.position.y = tool_pose.y;
  result->final_position.position.z = tool_pose.z;
  //this->alg_.unlock();
}

void KinovaLinearMovementAlgNode::kinova_linear_moveGetFeedbackCallback(iri_kinova_linear_movement::kinova_linear_movementFeedbackPtr& feedback)
{
  //this->alg_.lock();
  //update feedback data to be sent to client
  feedback->current_position.position.x = tool_pose.x;
  feedback->current_position.position.y = tool_pose.y;
  feedback->current_position.position.z = tool_pose.z;

  feedback->current_time = this->total_time.toSec();
  //ROS_INFO("feedback: ", feedback->current_position.position);
  //this->alg_.unlock();
}


/*  [action requests] */

void KinovaLinearMovementAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  if(config.rate!=this->getRate())
    this->setRate(config.rate);
  this->config_=config;
  this->alg_.unlock();
}

void KinovaLinearMovementAlgNode::addNodeDiagnostics(void)
{
}

//kortex_driver::TwistCommand KinovaLinearMovementAlgNode::get_velocity(const tf::Vector3& error, const tf::Vector3& ac_error){
kortex_driver::TwistCommand KinovaLinearMovementAlgNode::get_velocity(const tf::Vector3& error){
  tf::Vector3 desired_vel(0.0, 0.0, 0.0);
  float kp = 1.8;    // factor proporcional
  float ki = 0.0005;  // factor integral
  kortex_driver::TwistCommand twist_cmd;

  //desired_vel = error * kp + ac_error * ki;
  desired_vel = error * kp;
  if (desired_vel.x() > this->maximum_vel)
    desired_vel.setX(this->maximum_vel);
  else if (desired_vel.x() < -this->maximum_vel)
    desired_vel.setX(-this->maximum_vel);

  if (desired_vel.y() > this->maximum_vel)
    desired_vel.setY(this->maximum_vel);
  else if (desired_vel.y() < -this->maximum_vel)
    desired_vel.setY(-this->maximum_vel);

  if (desired_vel.z() > this->maximum_vel)
    desired_vel.setZ(this->maximum_vel);
  else if (desired_vel.z() < -this->maximum_vel)
    desired_vel.setZ(-this->maximum_vel);

  twist_cmd.reference_frame = this->twist_cmd.reference_frame;
  twist_cmd.duration = 0;
  twist_cmd.twist.linear_x = desired_vel.x();
  twist_cmd.twist.linear_y = desired_vel.y();
  twist_cmd.twist.linear_z = desired_vel.z();

  return twist_cmd;
}
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<KinovaLinearMovementAlgNode>(argc, argv, "kinova_linear_movement_alg_node");
}
