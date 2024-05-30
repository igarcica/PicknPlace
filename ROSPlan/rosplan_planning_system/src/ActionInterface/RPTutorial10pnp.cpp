#include "rosplan_action_interface/RPTutorial10.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <activatesm/activateSMAction.h>

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	RPTutorialInterface::RPTutorialInterface(ros::NodeHandle &nh) {
		// perform setup
		actionlib::SimpleActionClient<activatesm::activateSMAction> ac("activatesm", true);
	}

	/* action dispatch callback */
	bool RPTutorialInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
        std::cout << "HOLAA " << msg->action_id << std::endl;

        for(size_t i=0; i<msg->parameters.size(); i++) {
            std::cout << "keys: " << msg->parameters[i].key << std::endl;
            std::cout << "values: " << msg->parameters[i].value << std::endl;
        }
        
        // Call an action (created in the Demo?) that activates a section of the SM and returns a 
        // success state when finished (or failure if something went wrong in the SM)

		//FIBONACCI test
		actionlib::SimpleActionClient<activatesm::activateSMAction> ac("activatesm", true);

		ROS_INFO("Waiting for action server to start.");
  		// wait for the action server to start
  		ac.waitForServer(); //will wait for infinite time

  		ROS_INFO("Action server started, sending goal.");
  		// send a goal to the action
  		activatesm::activateSMGoal goal;
  		goal.order = 20;
  		ac.sendGoal(goal);

  		//wait for the action to return
  		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  		if (finished_before_timeout)
  		{
    		actionlib::SimpleClientGoalState state = ac.getState();
  	  		ROS_INFO("Action finished: %s",state.toString().c_str());
  		}
  		else
   			ROS_INFO("Action did not finish before the time out.");

		//END FIBONACCI test


		// complete the action
		ROS_INFO("KCL: (%s) TUTORIAL Action completing.", msg->name.c_str());
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_tutorial_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPTutorialInterface rpti(nh);
		


		rpti.runActionInterface();

		return 0;
	}