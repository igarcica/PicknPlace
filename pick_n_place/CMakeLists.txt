cmake_minimum_required(VERSION 2.8.3)
project(pick_n_place)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ********************************************************************
#                 Add catkin additional components here
# ********************************************************************
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm iri_kinova_linear_movement kortex_driver vision_pick_place iri_action_server message_generation std_msgs actionlib_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ********************************************************************
#           Add system and labrobotica dependencies here
# ********************************************************************
# find_package(<dependency> REQUIRED)

# ********************************************************************
#           Add topic, service and action definition here
# ********************************************************************
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#   demo_kinova.action
#)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs actionlib_msgs  # Or other packages containing msgs
 )

# ********************************************************************
#                 Add the dynamic reconfigure file
# ********************************************************************
generate_dynamic_reconfigure_options(cfg/PicknPlace.cfg)

# ********************************************************************
#                 Add run time dependencies here
# ********************************************************************
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
# ********************************************************************
#            Add ROS and IRI ROS run time dependencies
# ********************************************************************
 CATKIN_DEPENDS iri_base_algorithm iri_kinova_linear_movement vision_pick_place iri_action_server kortex_driver tf
# ********************************************************************
#      Add system and labrobotica run time dependencies here
# ********************************************************************
#  DEPENDS
)

###########
## Build ##
###########

# ********************************************************************
#                   Add the include directories
# ********************************************************************
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIRS})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/pick_n_place_alg.cpp src/pick_n_place_alg_node.cpp)

# ********************************************************************
#                   Add the libraries
# ********************************************************************
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARIES})

# ********************************************************************
#               Add message headers dependencies
# ********************************************************************
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_kinova_linear_movement_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} vision_pick_place_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} kortex_driver_generate_messages_cpp)
# ********************************************************************
#               Add dynamic reconfigure dependencies
# ********************************************************************
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
