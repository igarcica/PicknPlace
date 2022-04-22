
#include "pick_garment_demo_alg_node.h"

PickGarmentDemoAlgNode::PickGarmentDemoAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PickGarmentDemoAlgorithm>(),
  gripper("gripper_module",ros::this_node::getName()),
  arm("arm_module",ros::this_node::getName()),
  torso("torso_module",ros::this_node::getName()),
  head("head_module",ros::this_node::getName()),
  play_motion("play_motion_module",ros::this_node::getName()),
  move_platform("move_platform_module",ros::this_node::getName())
{

/* Garment pose subscriber */
  this->garment_pose_subscriber = this->public_node_handle_.subscribe("garment_pose",1,&PickGarmentDemoAlgNode::garment_pose_callback,this);

/* Marker pose publisher - grasp point */
  this->marker_pose_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("garment_marker", 1);
  this->marker_pose_timer = this->public_node_handle_.createTimer(ros::Duration(1.0),&PickGarmentDemoAlgNode::marker_pose_pub,this);
  this->marker_pose_timer.stop();

/* Publish new frames created (finger's tip and garment's corner) */
  this->new_frames_pub_timer = this->public_node_handle_.createTimer(ros::Duration(1.0),&PickGarmentDemoAlgNode::new_frames_pub,this);
  this->new_frames_pub_timer.stop();

/* SM Variables */
  this->state=IDLE;
  this->status=SUCCESS;
  this->start = false;
  this->start_platform = false;
  this->start_give = false;
  this->stop = false;
 
/* Saves detected grasping position */
  this->get_garment_pose=false;
 
/* Torso position before grasping */
  this->torso_position=0.34;

}

PickGarmentDemoAlgNode::~PickGarmentDemoAlgNode(void)
{
  // [free dynamic memory]
}

void PickGarmentDemoAlgNode::mainNodeThread(void)
{
  geometry_msgs::QuaternionStamped quat;

  /* Stop all process in execution */
  if(this->state!=IDLE && this->stop)
  {
    ROS_INFO("PickGarmentDemo: Grasp stopped");
    this->gripper.stop();
    this->arm.stop();
    this->torso.stop();
    this->head.stop();
    this->play_motion.stop();
    this->move_platform.stop();
    this->state=END;
    this->stop = false;
  }
  else
  {
    switch(this->state)
    {
      case IDLE: ROS_DEBUG("PickGarmentDemo: state IDLE"); 
                 if(this->start) /* Start SM from beginning */
                 {
                   //move torso down to see the table legs?
                   //this->torso.move_torso(0.24,2); //move torso up
                   /* Clear occupancy map */
                   this->arm.remove_object_from_environment("garment");
                   this->arm.remove_object("garment");
                   this->gripper.open();
                   /* Scan environment to build occupancy map */
                   this->set_head_scan_angles();
                   this->head.move_to(this->pan_angles,this->tilt_angles,this->durations);
                   /* Start SM */
                   this->start=false;
                   this->state=LOOK_DOWN;
                   ROS_INFO("PickGarmentDemo: state LOOK DOWN");
                 }
                 else if(this->start_give) /* Start SM with two corners grasped */
                 {
                    this->start_give=false;
                    this->play_motion.execute_motion("set_position"); /* Go to position for moving platform*/
                    this->state=SET_POSITION;
                    ROS_INFO("PickGarmentDemo: state SET POSITION");
                 }
                 else if(this->start_platform) /* Move platform to spread tablecloth*/
                 {
                    this->start_platform=false;
                    this->move_platform.move_platform(1.7);
                    this->state=MOVE_PLATFORM;
                    ROS_INFO("PickGarmentDemo: state MOVE PLATFORM");
                 }
                 else
                   this->state=IDLE;
      break;
    

      case LOOK_AROUND: ROS_DEBUG("PickGarmentDemo: state LOOK_AROUND");
                        if(this->head.is_finished()) /* Wait until scan ends*/
                        {
                          this->gripper.open();
                          this->torso.move_torso(this->torso_position,1);
                          this->state=LOOK_DOWN;
                          ROS_INFO("PickGarmentDemo: state LOOK DOWN");
                        }
                        else
                          this->state=LOOK_AROUND;
      break;

      case LOOK_DOWN: ROS_DEBUG("PickGarmentDemo: state LOOK DOWN");
                   /* Move torso up and head down to look at the table (no wall) */
                   if(this->torso.is_finished() && this->gripper.is_finished() && this->head.is_finished())
                   {
                     this->tf_timeout.start(ros::Duration(5.0));
                     this->grasp_pose.header.stamp=ros::Time::now();
                     this->state=GET_GRASP_TARGETS;
                     ROS_INFO("PickGarmentDemo: state GET GRASP TARGETS");
                   }
                   else
                     this->state=LOOK_DOWN;
      break;

      case GET_GRASP_TARGETS: ROS_DEBUG("PickGarmentDemo: state GET GRASP TARGETS");
                              if(this->tf_timeout.timed_out()) /* Stop SM if no transforms have been found */
                              {
                                ROS_WARN("PickGarmentDemo: Timed out");
                                this->status=NO_TRANSFORM;
                                this->state=END;
                                ROS_INFO("PickGarmentDemo: state END");
                              }
                              else if(this->compute_grasp_arm_targets()) /* Computes grasping positions (pregrasp, grasp, postgrasp) */
                              {
                                this->tf_timeout.stop();
                                this->get_garment_pose=false; /* Stop getting garment pose */
                                std::cout << "\033[1;32m Current position target -> \033[1;32m  x: " << this->pregrasp_arm_target.pose.position.x << ", y: " << this->pregrasp_arm_target.pose.position.y << ", z: " << this->pregrasp_arm_target.pose.position.z << std::endl;
                                this->arm.clear_path_constraints(); /* Clear planning constraints */
                                this->arm.move_to(this->pregrasp_arm_target,this->config.position_tol,this->config.orientation_tol); /* Move to pregrasp position */
                                ROS_INFO("PickGarmentDemo: state WAIT PREGRASP");
                                this->state=WAIT_PREGRASP;
                              }
                              else
                                this->state=GET_GRASP_TARGETS;
      break;

      case WAIT_PREGRASP: ROS_DEBUG("PickGarmentDemo: state WAIT PREGRASP");
                          if(this->arm.is_finished())// && this->config.go) /* Wait until arriving to pregrasp position */
                          {
                            this->config.go=false;
                            /* Remove collision around corner */
                            this->arm.add_object("garment", this->grasp_pose, 0.03, 0.03, 0.03);
                            this->arm.add_object_to_environment("garment");
                            this->arm.disable_collision("garment");
                            /* Move closer to the grasp position - level height */
                            this->pregrasp_arm_target.pose.position.z -= this->config.pregrasp_offset_z; //-= 0.11;
                            //this->pregrasp_arm_target.pose.position.x = this->pregrasp_arm_target.pose.position.x + 0.05; 
                            std::cout << "\033[1;32m Current position target -> \033[1;32m  x: " << this->pregrasp_arm_target.pose.position.x << ", y: " << this->pregrasp_arm_target.pose.position.y << ", z: " << this->pregrasp_arm_target.pose.position.z << std::endl;
                            this->arm.move_to(this->pregrasp_arm_target,this->config.position_tol,this->config.orientation_tol);
                            this->state=WAIT_PREGRASP2;
                            ROS_INFO("PickGarmentDemo: state WAIT PREGRASP 2");
                          }
                          else
                            this->state=WAIT_PREGRASP;
      break;

      case WAIT_PREGRASP2: ROS_DEBUG("PickGarmentDemo: state WAIT PREGRASP2");
                          if(this->arm.is_finished())// && this->config.go)
                          {
                            this->config.go=false;
                            /* Add orientation constraint - Mantain same orientation */
                            //this->arm.clear_path_constraints();
                            quat.header=this->grasp_arm_target.header;
                            quat.quaternion=this->grasp_arm_target.pose.orientation;
                            this->arm.add_orientation_path_constraint(quat,this->config.orientation_tol);
                            /* Move to grasp position */
                            std::cout << "\033[1;32m Current position target -> \033[1;32m  x: " << this->grasp_arm_target.pose.position.x << ", y: " << this->grasp_arm_target.pose.position.y << ", z: " << this->grasp_arm_target.pose.position.z << std::endl;
                            this->arm.move_to(this->grasp_arm_target,this->config.position_tol,this->config.orientation_tol);
                            this->state=WAIT_GRASP;
                            ROS_INFO("PickGarmentDemo: state WAIT GRASP");
                          }
                          else
                            this->state=WAIT_PREGRASP2;
      break;

      case WAIT_GRASP: ROS_DEBUG("PickGarmentDemo: state WAIT GRASP");
                       if(this->arm.is_finished()) /* Wait until arriving to grasp position */
                       {
                         this->state=CLOSE_GRIPPER;
                         this->gripper.fingers_distance(0.02); 
                         ROS_INFO("PickGarmentDemo: state CLOSE GRIPPER");
                       }
                       else
                         this->state=WAIT_GRASP;
      break;

      case CLOSE_GRIPPER: ROS_DEBUG("PickGarmentDemo: state CLOSE GRIPPER");
                          if(this->gripper.is_finished() && this->arm.is_finished())// && this->config.go) /* Close gripper */
                          {
                            this->config.go=false;
                            /* Move to postgrasp position */
                            std::cout << "\033[1;32m Current position target -> \033[1;32m  x: " << this->postgrasp_arm_target.pose.position.x << ", y: " << this->postgrasp_arm_target.pose.position.y << ", z: " << this->postgrasp_arm_target.pose.position.z << std::endl;
                            this->arm.move_to(this->postgrasp_arm_target,this->config.position_tol,this->config.orientation_tol);
//                            this->state=WAIT_POSTGRASP;
                            this->state=END;
                            ROS_INFO("PickGarmentDemo: state WAIT POSTGRASP");
                          }
                          else
                            this->state=CLOSE_GRIPPER;
      break;

      case WAIT_POSTGRASP: ROS_DEBUG("PickGarmentDemo: state WAIT POST GRASP");
                           if(this->arm.is_finished()) /* Wait until arriving to post grasp position */
                           {
                             this->head.move_to(0.0,0.0,1.0); /* Move head to avoid detecting the garment as collision obstacle */
                             this->play_motion.execute_motion("pregive"); /* Move arm to a predefined position */
                             this->state=PREGIVE;
                             ROS_INFO("PickGarmentDemo: state GIVE GARMENT");
                           }
                           else
                             this->state=WAIT_POSTGRASP;
      break;

      case PREGIVE: ROS_DEBUG("PickGarmentDemo: state PREGIVE");
                    if(this->play_motion.is_finished() && this->head.is_finished())
                    {
                      this->play_motion.execute_motion("give_garment"); /* Move arm to an offering position for edge tracing */
                      this->state=GIVE_GARMENT;
                      ROS_INFO("PickGarmentDemo: state GIVE GARMENT");
                    }
                    else
                      this->state=PREGIVE;
      break;
                      
      case GIVE_GARMENT: ROS_DEBUG("PickGarmentDemo: state GIVE GARMENT");
                       if(this->play_motion.is_finished() && this->config.go)
                       {
                           this->config.go=false;
                           this->play_motion.execute_motion("set_position"); /* Move arm to a stable position for spreading the garment with the mobile platform */
                           this->arm.remove_object_from_environment("garment");
                           this->arm.remove_object("garment");
                           this->state=SET_POSITION;
                           ROS_INFO("PickGarmentDemo: state SET POSITION");
                       }
                       else
                           this->state=GIVE_GARMENT;
      break;

      case SET_POSITION: ROS_DEBUG("PickGarmentDemo: state SET POSITION");
                         if(this->play_motion.is_finished() && this->config.go)
                         {
                           this->move_platform.move_platform(1.7);//this->config.table_length); /* Move mobile platform forward*/
                           this->state=MOVE_PLATFORM;
                           ROS_INFO("PickGarmentDemo: state MOVE PLATFORM");
                         }
                         else
                           this->state=SET_POSITION;
      break;
                         
      case MOVE_PLATFORM: ROS_DEBUG("PickGarmentDemo: state MOVE PLATFORM");
                          if(this->move_platform.is_finished())
                          {
                            this->play_motion.execute_motion("final_position"); /* Move arm to a predefined position for releasing tablecloth */
                            this->state=FINAL_POSITION;
                            ROS_INFO("PickGarmentDemo: state FINAL POSITION");
                          }
                          else
                            this->state=MOVE_PLATFORM;
      break;

      case FINAL_POSITION: ROS_DEBUG("PickGarmentDemo: state FINAL POSITION");
                          if(this->play_motion.is_finished())
                          {
                            this->gripper.open(); /* Open gripper - Release tablecloth */
                            this->state=OPEN_GRIPPER;
                            ROS_INFO("PickGarmentDemo: state OPEN GRIPPER");
                          }
                          else
                            this->state=FINAL_POSITION;
      break;

      case OPEN_GRIPPER: ROS_DEBUG("PickGarmentDemo: state OPEN GRIPPER");
                         if(this->gripper.is_finished())
                         {
                           ROS_INFO("PickGarmentDemo: state END"); /* End SM */ 
                           this->state=END;
                         }
                         else
                           this->state=OPEN_GRIPPER;
      break;

      case END: ROS_DEBUG("PickGarmentDemo: state END");
                /* Reset variables to default, stop active modules, etc */
                if(this->gripper.is_finished() && arm.is_finished() && torso.is_finished() && head.is_finished() && play_motion.is_finished())
                {
                  this->state=IDLE;
                  ROS_INFO("PickGarmentDemo: state IDLE");
                }
                else
                  this->state=END;
      break;

    }
  }  
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PickGarmentDemoAlgNode::node_config_update(Config &config, uint32_t level)
{
  ROS_INFO("PickGarmentDemo: reconfigure callback");
  this->alg_.lock();

  /* Start task */
  if (config.start) 
  {
    this->get_garment_pose=true;
    this->set_config();
    this->start=true;
    config.start = false;
  }
  /* Stop task */
  if (config.stop) 
  {
    this->stop = true;
    config.stop = false;
  }
  /* Start task from gr2 (both corners already grasped) */
  if (config.start_platform) 
  {
    this->start_platform = true;
    config.start_platform = false;
  }
  /* Start task from gr1 (corner already grasped) */
  if (config.start_give) 
  {
    this->start_give = true;
    config.start_give = false;
  }
  /* Start the task with a given grasp point */
  else if(config.test)
  {
    /* Set grasp pose */
    this->grasp_pose.header.stamp=ros::Time::now();
    this->grasp_pose.header.frame_id=config.frame_id;
    this->grasp_pose.pose.position.x=config.x_pos;
    this->grasp_pose.pose.position.y=config.y_pos;
    this->grasp_pose.pose.position.z=config.z_pos;
    this->set_config();
    this->start=true;
    config.test=false;
  }
  this->config=config;
  this->alg_.unlock();
}

/* Starts grasp point marker and grasping frame topics with given rates */
void PickGarmentDemoAlgNode::set_config(void)
{
  ROS_INFO("PickGarmentDemoAlgNode: Set configuration");

  /* Start publisher rates */
  this->marker_pose_timer.setPeriod(ros::Duration(1.0/50.0));
  this->marker_pose_timer.start();
  this->new_frames_pub_timer.setPeriod(ros::Duration(1.0/50.0));
  this->new_frames_pub_timer.start();
}

/* Computes the poses in terms of "arm_tool_link" frame for moving the fingertip to the given grasping position */
bool PickGarmentDemoAlgNode::compute_grasp_arm_targets(void)
{
  geometry_msgs::PointStamped point;
  std::string target_frame_id="/base_footprint";
  double roll,pitch,yaw;
  tf::Quaternion q;
  tf::Vector3 v;
  tf::Transform T1,T2,T3;
  tf::Vector3 out,in(0.0,0.0,0.0);
  try{
      if(this->listener.canTransform("grasp_frame","arm_tool_link",this->grasp_pose.header.stamp))
      {
          tf::StampedTransform tmp;
          this->listener.lookupTransform("grasp_frame","arm_tool_link",this->grasp_pose.header.stamp,tmp);
          T1=tf::Transform(tmp.getBasis(),tmp.getOrigin());
          //std::cout << "ORIGIN: " << tmp.getOrigin().getX() << ", " << tmp.getOrigin().getY() << ", " << tmp.getOrigin().getZ()<< std::endl;
    
          point.header=this->grasp_pose.header;
          point.point=this->grasp_pose.pose.position;
          std::cout << "\033[1;36mGARMENT POSE-> \033[1;36m  x: " << grasp_pose.pose.position.x << ", y: " << grasp_pose.pose.position.y << ", z: " << grasp_pose.pose.position.z << std::endl;
      }
      else
      {
          //tf::StampedTransform tmp;
          //this->listener.lookupTransform("grasp_frame","arm_tool_link",this->grasp_pose.header.stamp,tmp);
          ROS_WARN("PickGarmentDemo: Transform not found!");
          return false;
      }
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
      return false;
  }
  
  //std::cout << "POINT-> x: " << point.point.x << ", y: " << point.point.y << ", z: " << point.point.z << std::endl;

  this->pregrasp_arm_target.header.frame_id=target_frame_id;
  this->grasp_arm_target.header.frame_id=target_frame_id;
  this->postgrasp_arm_target.header.frame_id=target_frame_id;
/* Given pregrasp target orientation */
  roll=config.roll;//1.414; //0.0;
  yaw=config.yaw;
  pitch=config.pitch;//0.0; //0.64;
  //yaw=atan2(-this->config.pregrasp_offset_y,-this->config.pregrasp_offset_x); 
  //pitch=atan2(-this->config.pregrasp_offset_z,-this->config.pregrasp_offset_x);
  std::cout << "\033[1;36mDesired orientation -> \033[1;36m Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
/* Grasp target position */
  this->pregrasp_arm_target.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  v=tf::Vector3(point.point.x,point.point.y,point.point.z);
  q=tf::Quaternion(0.0,0.0,0.0,1.0);
  T3=tf::Transform(q,v);
  v=tf::Vector3(this->config.pregrasp_offset_x,this->config.pregrasp_offset_y,this->config.pregrasp_offset_z);
  q=tf::Quaternion(this->pregrasp_arm_target.pose.orientation.x,this->pregrasp_arm_target.pose.orientation.y,this->pregrasp_arm_target.pose.orientation.z,this->pregrasp_arm_target.pose.orientation.w);
  T2=tf::Transform(q,v);
  out=T3*T2*T1*in;
  this->pregrasp_arm_target.pose.position.x=out.x();
  this->pregrasp_arm_target.pose.position.y=out.y();
  this->pregrasp_arm_target.pose.position.z=out.z();
  std::cout << "\033[1;36m Pregrasp position -> \033[1;36m  x: " << pregrasp_arm_target.pose.position.x << ", y: " << pregrasp_arm_target.pose.position.y << ", z: " << pregrasp_arm_target.pose.position.z << std::endl;
/* Grasp target position */
/* Grasp target orientation (keeps same orientation that pregrasp) */
  this->grasp_arm_target.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  v=tf::Vector3(0.0,0.0,0.0);
  q=tf::Quaternion(this->pregrasp_arm_target.pose.orientation.x,this->pregrasp_arm_target.pose.orientation.y,this->pregrasp_arm_target.pose.orientation.z,this->pregrasp_arm_target.pose.orientation.w);
  T2=tf::Transform(q,v);
  out=T3*T2*T1*in;
  this->grasp_arm_target.pose.position.x=out.x();
  this->grasp_arm_target.pose.position.y=out.y();
  this->grasp_arm_target.pose.position.z=out.z();
  std::cout << "\033[1;36m Grasp position -> \033[1;36m  x: " << grasp_arm_target.pose.position.x << ", y: " << grasp_arm_target.pose.position.y << ", z: " << grasp_arm_target.pose.position.z << std::endl;
/* Postgrasp target position */
/* Postgrasp target orientation (keep the same orientation) */
  this->postgrasp_arm_target.pose.position.x=point.point.x+this->config.postgrasp_offset_x;
  this->postgrasp_arm_target.pose.position.y=point.point.y+this->config.postgrasp_offset_y;
  this->postgrasp_arm_target.pose.position.z=point.point.z+this->config.postgrasp_offset_z;
  this->postgrasp_arm_target.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  v=tf::Vector3(this->config.postgrasp_offset_x,this->config.postgrasp_offset_y,this->config.postgrasp_offset_z);
  q=tf::Quaternion(this->pregrasp_arm_target.pose.orientation.x,this->pregrasp_arm_target.pose.orientation.y,this->pregrasp_arm_target.pose.orientation.z,this->pregrasp_arm_target.pose.orientation.w);
  T2=tf::Transform(q,v);
  out=T3*T2*T1*in;
  this->postgrasp_arm_target.pose.position.x=out.x();
  this->postgrasp_arm_target.pose.position.y=out.y();
  this->postgrasp_arm_target.pose.position.z=out.z();
  std::cout << "\033[1;36m Postgrasp position -> \033[1;36m  x: " << postgrasp_arm_target.pose.position.x << ", y: " << postgrasp_arm_target.pose.position.y << ", z: " << postgrasp_arm_target.pose.position.z << std::endl;

  return true;
}

/* Set head angles to scan environment */
void PickGarmentDemoAlgNode::set_head_scan_angles(void)
{

  /* Pan angles */
  this->pan_angles.clear();
  this->pan_angles.push_back(0.0); 
  this->pan_angles.push_back(0.7);
  this->pan_angles.push_back(0.7);
  this->pan_angles.push_back(0.0);
  this->pan_angles.push_back(-0.7);
  this->pan_angles.push_back(-0.7);
  this->pan_angles.push_back(0.0);
  this->pan_angles.push_back(0.7);
  this->pan_angles.push_back(0.6);
  /* Tilt angles */
  this->tilt_angles.clear();
  this->tilt_angles.push_back(0.0);
  this->tilt_angles.push_back(0.0);
  this->tilt_angles.push_back(-0.5);
  this->tilt_angles.push_back(-0.5);
  this->tilt_angles.push_back(-0.5);
  this->tilt_angles.push_back(-0.7);
  this->tilt_angles.push_back(-0.7);
  this->tilt_angles.push_back(-0.7);
  this->tilt_angles.push_back(-0.7);
  /* Head movement duration */
  this->head_duration=1;
  this->durations.clear();
  this->durations.push_back(1);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(this->head_duration);
  this->durations.push_back(1);
}

/* Topics callbacks and publishers */

/* Receives grasping position */
void PickGarmentDemoAlgNode::garment_pose_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  ROS_DEBUG("PickGarmentDemoAlgNode: garment pose callback");

  if(this->get_garment_pose)
  {
    this->grasp_pose.header.stamp=ros::Time::now();
    this->grasp_pose.header.frame_id=msg->header.frame_id;
    this->grasp_pose.pose.position=msg->pose.position;
  }
}

/* Create and publish finger tip and garment's corner frames */
void PickGarmentDemoAlgNode::new_frames_pub(const ros::TimerEvent& event)
{
  /* Create finger tip frame */
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.09,-0.013,-0.05));
  tf::Quaternion q;
  q.setRPY(-1.571,0,0);
  transform.setRotation(q);
  this->broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"gripper_grasping_frame","grasp_frame"));
}

/* Publishes marker to visualize the grasping position */
void PickGarmentDemoAlgNode::marker_pose_pub(const ros::TimerEvent& event)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_footprint";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = this->grasp_pose.pose.position.x;
  marker.pose.position.y = this->grasp_pose.pose.position.y;
  marker.pose.position.z = this->grasp_pose.pose.position.z;
  marker.scale.x=0.01;
  marker.scale.y=0.01;
  marker.scale.z=0.01;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  this->marker_pose_publisher.publish(marker);
}

void PickGarmentDemoAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PickGarmentDemoAlgNode>(argc, argv, "pick_garment_demo_alg_node");
}
