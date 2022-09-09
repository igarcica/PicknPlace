## Description

The iri_demos_kinova project description

# ROS Interface
### Action clients
  - ~**kinova_linear_move** (iri_kinova_linear_movement/kinova_linear_movement.action)
  - ~**kinova_linear_move** (iri_kinova_linear_movement/kinova_linear_movement.action)
### Action servers
  - ~**my_action** (iri_demos_kinova/demo_kinova.action)
### Service clients
  - ~**exec_wp_trajectory** (kortex_driver/ExecuteWaypointTrajectory.srv)
  - ~**base_execute_action** (kortex_driver/ExecuteAction.srv)
  - ~**base_read_action** (kortex_driver/ReadAction.srv)
  - ~**send_gripper_cmd** (kortex_driver/SendGripperCommand.srv)
  - ~**send_gripper_cmd** (kortex_driver/SendGripperCommand.srv)
  - ~**set_cartesian_rf** (kortex_driver/SetCartesianReferenceFrame.srv)
  - ~**base_clear_faults** (kortex_driver/Base_ClearFaults.srv)
  - ~**activate_publishing** (kortex_driver/OnNotificationActionTopic.srv)
### Topic subscribers
  - ~**base_feedback** (kortex_driver/BaseCyclic_Feedback.msg)
  - ~**action_topic** (kortex_driver/ActionNotification.msg)
### Topic publishers
  - ~**cartesian_velocity** (kortex_driver/TwistCommand.msg)
  - ~**my_gen3_action_topic** (kortex_driver/ActionNotification.msg)

### Parameters
- ~**rate** (Double; default: 10.0; min: 0.1; max: 1000) The main node thread loop rate in Hz. 

## Installation

Move to the active workspace:
```bash
roscd && cd ../src
```
Clone the repository: 
```bash
git clone <url>
```
Install ROS dependencies:
```
roscd
cd ..
rosdep install -i -r --from-paths src
```
Compile the workspace:
```
catkin_make
```

## How to use it

- Standalone test

  `roslaunch iri_demos_kinova test.launch`

## Disclaimer  

Copyright (C) Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Mantainer IRI labrobotics (labrobotica@iri.upc.edu)

This package is distributed in the hope that it will be useful, but without any warranty. It is provided "as is" without warranty of any kind, either expressed or implied, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose. The entire risk as to the quality and performance of the program is with you. should the program prove defective, the GMR group does not assume the cost of any necessary servicing, repair  or correction.

In no event unless required by applicable law the author will be liable to you for damages, including any general, special, incidental or consequential damages arising out of the use or inability to use the program (including but not limited to loss of data or data being rendered inaccurate or losses sustained by you or third parties or a failure of the program to operate with any other programs), even if the author has been advised of the possibility of such damages.

You should have received a copy of the GNU Lesser General Public License along with this program. If not, see <http://www.gnu.org/licenses/>