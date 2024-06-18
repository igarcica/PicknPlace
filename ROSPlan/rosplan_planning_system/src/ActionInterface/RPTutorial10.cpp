#include "rosplan_action_interface/RPTutorial10.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <activatesm/activateSMAction.h>
#include <pick_n_place/activateSMAction.h>

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	RPTutorialInterface::RPTutorialInterface(ros::NodeHandle &nh) {
		// perform setup
		//actionlib::SimpleActionClient<pick_n_place::activateSMAction> ac("pick_n_place/activatesm", true);
	}

	/* action dispatch callback */
	bool RPTutorialInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
		//actionlib::SimpleActionClient<activatesm::activateSMAction> ac("activatesm", true);
		actionlib::SimpleActionClient<pick_n_place::activateSMAction> ac("/pick_n_place/activatesm", true);

        std::cout << "HOLA " << msg->action_id << std::endl;
		ROS_WARN("RPTutorial: Sending PDDL action (%s)", msg->name.c_str());

        for(size_t i=0; i<msg->parameters.size(); i++) {
            std::cout << "keys: " << msg->parameters[i].key << std::endl;
            std::cout << "values: " << msg->parameters[i].value << std::endl;
        }
        
        // Call an action (created in the Demo?) that activates a section of the SM and returns a 
        // success state when finished (or failure if something went wrong in the SM)

		ROS_INFO("RPTutorial: Waiting for action server to start.");
  		ac.waitForServer(); //wait for the action server to start. will wait for infinite time
  		ROS_INFO("RPTutorial: Action server started, sending goal.");

		//Activate SM action test
		//activatesm::activateSMGoal goal;
		pick_n_place::activateSMGoal goal;
  		goal.action_name = msg->name.c_str();
		goal.activate_grasp = true;
  		ac.sendGoal(goal);

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

		if (finished_before_timeout)
  		{
    		actionlib::SimpleClientGoalState state = ac.getState();
  	  		ROS_INFO("RPTutorial: Action finished: %s",state.toString().c_str());
			//pick_n_place::activateSMResult result = ac.getResult();
			//actionlib::ClientGoalHandle result = ac.getResult();
			//actionlib::SimpleActionClient result = ac.getResult();
			//actionlib::SimpleActionClient<pick_n_place::activatesmAction> result = ac.getResult();

			pick_n_place::activateSMResultConstPtr result = ac.getResult();
			std::cout << int(result->done_action) << std::endl;
			if(int(result->done_action))
				ROS_INFO("RPTutorial: Action ended successfully.");
			else
				ROS_INFO("RPTutorial: Action failed.");

			return int(result->done_action);
			//return true;
  		}
  		else
		{
   			ROS_INFO("RPTutorial: Action did not finish before the time out.");
			ac.cancelGoal();
			return false;
		}

		//FIBONACCI test
  		// send a goal to the action
  		// pick_n_place::activateSMGoal goal;
  		// goal.order = 20;
  		// ac.sendGoal(goal);

  		// //wait for the action to return
  		// bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  		// if (finished_before_timeout)
  		// {
    	// 	actionlib::SimpleClientGoalState state = ac.getState();
  	  	// 	ROS_INFO("Action finished: %s",state.toString().c_str());
  		// }
  		// else
   		// 	ROS_INFO("Action did not finish before the time out.");

		//END FIBONACCI test


		// complete the action
		ROS_INFO("RPTutorial: (%s) TUTORIAL Action completing.", msg->name.c_str());
		//return true;
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