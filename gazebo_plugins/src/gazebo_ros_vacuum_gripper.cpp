/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboVacuumGripper plugin for manipulating objects in Gazebo
   Author: Kentaro Wada
   Date: 7 Dec 2015
 */

#include <algorithm>
#include <assert.h>

#include <std_msgs/Bool.h>
#include <gazebo_plugins/gazebo_ros_vacuum_gripper.h>


namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVacuumGripper::GazeboRosVacuumGripper()
{
  connect_count_ = 0;
  status_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVacuumGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_NAMED("vacuum_gripper", "Loading gazebo_ros_vacuum_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // Realtive pose not initialized 
  rel_pose_init_ = false;

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_);
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("vacuum_gripper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&GazeboRosVacuumGripper::Connect, this),
    boost::bind(&GazeboRosVacuumGripper::Disconnect, this),
    ros::VoidPtr(), &queue_);
  pub_ = rosnode_->advertise(ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "on", boost::bind(&GazeboRosVacuumGripper::OnServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "off", boost::bind(&GazeboRosVacuumGripper::OffServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVacuumGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVacuumGripper::UpdateChild, this));

  ROS_INFO_NAMED("vacuum_gripper", "Loaded gazebo_ros_vacuum_gripper");
}

bool GazeboRosVacuumGripper::OnServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'on'");
  } else {
    status_ = true;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: off -> on");
  }
  return true;
}
bool GazeboRosVacuumGripper::OffServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: on -> off");
  } else {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'off'");
  }
  rel_pose_init_ = false;
  current_picked_model_->SetStatic(false);
  current_picked_model_->SetGravityMode(true);
  ROS_INFO_STREAM_NAMED("vacuum_gripper", "current_picked_model_ name released:  "<< current_picked_model_->GetName() );
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVacuumGripper::UpdateChild()
{
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
    pub_.publish(grasping_msg);
    return;
  }
  // apply force
  lock_.lock();
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d parent_pose = link_->WorldPose();
  //Search the models and put them into a vector
  physics::Model_V models = world_->Models();
#else
  ignition::math::Pose3d parent_pose = link_->GetWorldPose().Ign();
  physics::Model_V models = world_->GetModels();
#endif
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: link_->GetName(): "<< link_->GetName() );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: parent_->GetName(): "<< parent_->GetName() );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: models.size(): "<< models.size() );
  //Distance between the vacuum gripper link and the model link in this loop
  //used to determine the closests model to the gripper
  //Initialised at 1.0 meter
  double current_norm = 1.0;
  ignition::math::Pose3d current_link_pose;
  int current_model_index;
  bool box_found = 0;
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName() ||
        models[i]->GetName() == "ground_plane" )
    {
      //Exclude the models in the vector with the same name as robot vacuum gripper name and the robot name. 
      //This is to avoid interference between the vacuum gripper and robot links
      continue;
    }
    //The model to carry by the suction gripper can contain different links so use a vector to contain the links of the model
    physics::Link_V links = models[i]->GetLinks();
    //ROS_INFO_STREAM_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: links.size(): "<< links.size() );
    for (size_t j = 0; j < links.size(); j++) {
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d link_pose = links[j]->WorldPose();
#else
      ignition::math::Pose3d link_pose = links[j]->GetWorldPose().Ign();
#endif
      ignition::math::Pose3d diff = parent_pose - link_pose;
      double norm = diff.Pos().Length();
      //Distance between the vacuum gripper link and the model link in this loop
      //Search for the smallest link
      if (norm < 0.06 and norm < current_norm) {
        grasping_msg.data = true;
        box_found = 1;
        current_norm = norm;
	current_link_pose = link_pose;
	current_model_index = i;
      }
    }
  }

  //Set static pose off the model in the link frame
  if(rel_pose_init_ == false and box_found == 1 ){
    relative_pose_ = - parent_pose * current_link_pose;
    rel_pose_init_ = true;
    ROS_INFO_STREAM_NAMED("vacuum_gripper", "current_link_pose : "<< current_link_pose );
    ROS_INFO_STREAM_NAMED("vacuum_gripper", "parent_pose : "<< parent_pose );
    ROS_INFO_STREAM_NAMED("vacuum_gripper", "relative_pose : "<< relative_pose_ );
    current_picked_model_ = models[current_model_index];
    ROS_INFO_STREAM_NAMED("vacuum_gripper", "current_picked_model_ : "<< current_picked_model_->GetName() );
  }

  if(rel_pose_init_ == true){
  ignition::math::Pose3d box_pose;
  //box_pose.Set( parent_pose.Pos() - relative_pose_.Pos(), parent_pose.Rot() - relative_pose_.Rot());
  box_pose = parent_pose * relative_pose_;
  current_picked_model_->SetStatic(true);
  current_picked_model_->SetGravityMode(false);
  current_picked_model_->SetWorldPose(box_pose);
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "links[j]->GetName(): "<< links[j]->GetName() );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "box_pose : "<< box_pose );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "parent_pose : "<< parent_pose );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "relative_pose_ : "<< relative_pose_ );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "parent_pose + relative_pose_ : "<< parent_pose + relative_pose_ );
  //ROS_INFO_STREAM_NAMED("vacuum_gripper", "parent_pose - relative_pose_ : "<< parent_pose - relative_pose_ );
  }
  pub_.publish(grasping_msg);
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVacuumGripper::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Connect()
{
  this->connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Disconnect()
{
  this->connect_count_--;
}

}  // namespace gazebo
