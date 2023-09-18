// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cyberdog_visual/cyberdog_visual.hpp"

using namespace std;
using namespace urdf;

typedef shared_ptr<urdf::Link> LinkPtr;
typedef const shared_ptr<const urdf::Link> ConstLinkPtr;
typedef shared_ptr<urdf::Joint> JointPtr;
typedef shared_ptr<urdf::ModelInterface> ModelPtr;
typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;


#define param_double(a,b,c) \
          this->declare_parameter((a), (c));\
          (b) = get_parameter(a).as_double();\

#define param_string(a,b,c) \
          this->declare_parameter((a), (c));\
          (b) = get_parameter(a).as_string();\

#define param_bool(a,b,c) \
          this->declare_parameter((a), (c));\
          (b) = get_parameter(a).as_bool();\

namespace cyberdog
{
	CyberDogVisual::CyberDogVisual():Node("CyberDogVisual"),odom_lcm_("udpm://239.255.76.67:7669?ttl=255"),odom_global_lcm_("udpm://239.255.76.67:7667?ttl=255"),
	joint_lcm_("udpm://239.255.76.67:7667?ttl=255")
	{
		param_double("publish_frequency", publish_frequency_, 0.0);
		param_string("joint_state_topic", joint_state_topic_, "");
		param_string("robot_description", urdf_string, "");
		param_bool("use_state_estimator", use_state_estimator_, false);
		ReadParameters();

		//js_pub_ = n_.advertise<sensor_msgs::JointState>(joint_state_topic_, 5);
		js_pub_ =  this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::SystemDefaultsQoS());
		br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	

		if (!get_parameter("robot_description", urdf_string)) {
			RCLCPP_ERROR(this->get_logger(),"Failed to get param 'robot_description' " );
		}
		urdf::Model urdf_model;
		urdf_model.initString(urdf_string);
		root_link_ = urdf_model.getRoot()->name;

		js_.name = GetJointName(urdf_model);
		js_.position.resize(js_.name.size());
		js_.velocity.resize(js_.name.size());
		js_.effort.resize(js_.name.size());

		if (!odom_lcm_.good()||!joint_lcm_.good()) {
			exit(1);
		}

		if(use_state_estimator_){
			odom_lcm_.subscribe("state_estimator", &CyberDogVisual::HandleOdomMessage, this);
		}
		else{
			odom_global_lcm_.subscribe("global_to_robot", &CyberDogVisual::HandleGlobalOdomMessage, this);
		}

		joint_lcm_.subscribe("leg_control_data", &CyberDogVisual::HandleJointMessage, this);
		
		tf2::Duration check_period = tf2::durationFromSec(1/publish_frequency_);
		update_timer_ = rclcpp::create_timer(
          this, this->get_clock(), check_period, std::bind(&CyberDogVisual::UpdateTimerCallback, this));
		RCLCPP_INFO(this->get_logger(), "Legged visual started.");
	}

	CyberDogVisual::~CyberDogVisual()
	{
	}

	//publishes output in given fixed rate
	void CyberDogVisual::UpdateTimerCallback()
	{
		if(use_state_estimator_){
			odom_lcm_.handle();
		}
		else{
			odom_global_lcm_.handle();
		}
		joint_lcm_.handle();
	}

	void CyberDogVisual::HandleOdomMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const state_estimator_lcmt *msg)
	{
		(void) rbuf;
		(void) chan;

		//tf::Transform transform;
		geometry_msgs::msg::TransformStamped transform;

		//transform.setOrigin(tf::Vector3(msg->p[0], msg->p[1], msg->p[2]));
		transform.transform.translation.x = msg->p[0];
		transform.transform.translation.y = msg->p[1];
		transform.transform.translation.z = msg->p[2];

		//tf::Quaternion q;
		// q.setValue(msg->base_quat[0], msg->base_quat[1], msg->base_quat[2], msg->base_quat[3]);
		
		tf2::Quaternion q(msg->quat[1], msg->quat[2], msg->quat[3], msg->quat[0]);
		geometry_msgs::msg::Quaternion geoQuat;
		tf2::convert(q, geoQuat);

		//q.setValue( msg->quat[1], msg->quat[2], msg->quat[3], msg->quat[0]);
		//transform.setRotation(q);
		transform.transform.rotation = geoQuat;
		transform.header.stamp = this->get_clock()->now();
		transform.header.frame_id  = "vodom";
		transform.child_frame_id = root_link_;
		//br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", root_link_));
		br_->sendTransform(transform);
	}

	void CyberDogVisual::HandleGlobalOdomMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const localization_lcmt *msg){

		// std::cout << "handling message" << std::endl;
		// RCLCPP_INFO(this->get_logger(), "handling message.");
		(void) rbuf;
		(void) chan;

		//tf::Transform transform;
		geometry_msgs::msg::TransformStamped transform;

		//transform.setOrigin(tf::Vector3(msg->p[0], msg->p[1], msg->p[2]));
		transform.transform.translation.x = msg->xyz[0];
		transform.transform.translation.y = msg->xyz[1];
		transform.transform.translation.z = msg->xyz[2];

		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(msg->rpy[0],Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(msg->rpy[1],Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(msg->rpy[2],Eigen::Vector3d::UnitZ()));
		
		Eigen::Quaterniond quaternion;
		quaternion = yawAngle * pitchAngle * rollAngle;

		tf2::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(),quaternion.w());
		geometry_msgs::msg::Quaternion geoQuat;
		tf2::convert(q, geoQuat);

		transform.transform.rotation = geoQuat;
		transform.header.stamp = this->get_clock()->now();
		transform.header.frame_id  = "vodom";
		transform.child_frame_id = root_link_;
		//br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", root_link_));
		br_->sendTransform(transform);
	}

	void CyberDogVisual::HandleJointMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const leg_control_data_lcmt *msg)
	{
		(void) rbuf;
		(void) chan;
		
		js_.header.stamp = this->get_clock()->now();
		for (uint i = 0; i < js_.name.size(); i++) {
			js_.effort[i] = 0.0;
		}
		uint leg_map[4] = {1, 0, 3, 2};
		for (uint i = 0; i < 4; i++) {
				js_.position[3 * i + 0] = msg->q[3*leg_map[i]+0];
				js_.position[3 * i + 1] = -msg->q[3*leg_map[i]+1];
				js_.position[3 * i + 2] = -msg->q[3*leg_map[i]+2];
				js_.velocity[3 * i + 0] = msg->qd[3*leg_map[i]+0];
				js_.velocity[3 * i + 1] = -msg->qd[3*leg_map[i]+1];
				js_.velocity[3 * i + 2] = -msg->qd[3*leg_map[i]+2];
				js_.effort[3 * i + 0] = msg->tau_est[3*leg_map[i]+0];
				js_.effort[3 * i + 1] = -msg->tau_est[3*leg_map[i]+1];
				js_.effort[3 * i + 2] = -msg->tau_est[3*leg_map[i]+2];
		}
		js_pub_->publish(js_);
	}

	void CyberDogVisual::ReadParameters()
	{
		publish_frequency_ = get_parameter("publish_frequency").as_double();
		joint_state_topic_ = get_parameter("joint_state_topic").as_string();
	}

	vector<string> CyberDogVisual::GetJointName(const urdf::Model &urdf_model)
	{
		URDFLinkMap link_map;
		link_map = urdf_model.links_;

		URDFJointMap joint_map;
		joint_map = urdf_model.joints_;

		vector<string> joint_names;

		stack<LinkPtr> link_stack;
		stack<int> joint_index_stack;

		// add the bodies in a depth-first order of the model tree
		link_stack.push(link_map[(urdf_model.getRoot()->name)]);

		if (link_stack.top()->child_joints.size() > 0) {
			joint_index_stack.push(0);
		}

		while (link_stack.size() > 0) {
			LinkPtr cur_link = link_stack.top();
			unsigned int joint_idx = joint_index_stack.top();

			if (joint_idx < cur_link->child_joints.size()) {
				JointPtr cur_joint = cur_link->child_joints[joint_idx];

				// increment joint index
				joint_index_stack.pop();
				joint_index_stack.push(joint_idx + 1);

				link_stack.push(link_map[cur_joint->child_link_name]);
				joint_index_stack.push(0);

				if (cur_joint->type == urdf::Joint::REVOLUTE)
				joint_names.push_back(cur_joint->name);
			}
			else {
				link_stack.pop();
				joint_index_stack.pop();
			}
		}
		return joint_names;
	}

}