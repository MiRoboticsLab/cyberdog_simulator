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

#include "legged_plugin.hpp"

namespace gazebo
{

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeggedPlugin)

  _contact_force GetContactForce(const msgs::Contacts &contacts)
  { 
    Eigen::Vector3d force;
    std::string parent_name;
    unsigned int count_ = contacts.contact_size();
    for (unsigned int i = 0; i < count_; ++i) {

      if (contacts.contact(i).position_size() != 1) {

        std::cerr << "Contact count isn't correct!!!!" << std::endl;
      }

      for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
        force[0] += contacts.contact(i).wrench(0).body_1_wrench().force().x(); // Notice: the force is in local coordinate, not in world or base coordnate.
        force[1] += contacts.contact(i).wrench(0).body_1_wrench().force().y();
        force[2] += contacts.contact(i).wrench(0).body_1_wrench().force().z();
      }
            
    }
    
    if (count_ != 0) {
      
      force[0] = force[0] / double(count_);
      force[1] = force[1] / double(count_);
      force[2] = force[2] / double(count_);

      
      
      parent_name = contacts.contact(0).wrench(0).body_1_name();
      int index = parent_name.find("::");
      parent_name = parent_name.substr(index+2,parent_name.length());
      index = parent_name.find("::");
      parent_name = parent_name.substr(0,index);  
    }
    else {
      force[0] = 0;
      force[1] = 0;
      force[2] = 0;
      parent_name = "";
    }
     
    return {force, parent_name};
  }

  void LeggedPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    std::cout << "**************Enter plugin**************" << std::endl;
    // Store the pointer to the model
    model_ = _parent;
    
    std::cout<<model_->GetName()<<" is import"<<std::endl;

    // Initialize node executor the recieve topic messages 
    node_executor_ = new NodeExc();

    force_node_ = std::make_shared<GazeboNode>("force_node");
    for_sub_ = force_node_->create_subscription<cyberdog_msg::msg::ApplyForce>("apply_force", 10, std::bind(&LeggedPlugin::ForceHandler,this,std::placeholders::_1));
    node_executor_->AddNode(force_node_);

    // Initialize the sender and recieve of simulator parameters
    simparam_ = new SimParam(model_->GetName(),node_executor_);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&LeggedPlugin::OnUpdate, this));

    // get the list of sensors
    sensors_ = gazebo::sensors::SensorManager::Instance()->GetSensors();

    // if multiple robot are simulated we need to get only the sensors attached to our robot
    for (unsigned int i = 0; i < sensors_.size(); ++i){
      if (sensors_[i]->ScopedName().find("::" + model_->GetName() + "::") != std::string::npos)
      {
        sensors_attached_to_robot_.push_back(sensors_[i]);
        std::cout << sensors_attached_to_robot_[i]->ScopedName() << std::endl;
      }
    }

    for (unsigned int i = 0; i < sensors_attached_to_robot_.size(); ++i) {
      if (sensors_attached_to_robot_[i]->Type().compare("imu") == 0) {
        imu_sensor_ = std::static_pointer_cast<gazebo::sensors::ImuSensor>(sensors_attached_to_robot_[i]);
        std::cout << "IMU found: " << imu_sensor_->Name() << std::endl;
      }
      if (sensors_attached_to_robot_[i]->Name().compare("FL_foot_contact") == 0) {
        contact_sensor_fl_ = std::static_pointer_cast<gazebo::sensors::ContactSensor>(sensors_attached_to_robot_[i]);
        std::cout << "Contact sensor found: " << contact_sensor_fl_->Name() << std::endl;
      }
      if (sensors_attached_to_robot_[i]->Name().compare("FR_foot_contact") == 0) {
        contact_sensor_fr_ = std::static_pointer_cast<gazebo::sensors::ContactSensor>(sensors_attached_to_robot_[i]);
        std::cout << "Contact sensor found: " << contact_sensor_fr_->Name() << std::endl;
      }
      if (sensors_attached_to_robot_[i]->Name().compare("RL_foot_contact") == 0) {
        contact_sensor_hl_ = std::static_pointer_cast<gazebo::sensors::ContactSensor>(sensors_attached_to_robot_[i]);
        std::cout << "Contact sensor found: " << contact_sensor_hl_->Name() << std::endl;
      }
      if (sensors_attached_to_robot_[i]->Name().compare("RR_foot_contact") == 0) {
        contact_sensor_hr_ = std::static_pointer_cast<gazebo::sensors::ContactSensor>(sensors_attached_to_robot_[i]);
        std::cout << "Contact sensor found: " << contact_sensor_hr_->Name() << std::endl;

      }
      contact_ = std::vector<double>(4, 1.0);
    }

    // iterate over Gazebo model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V &joints = model_->GetJoints();
    for (unsigned int i = 0; i < joints.size(); i++) {
      std::string joint_name = joints[i]->GetName();
      joint_names_.push_back(joint_name);
      joint_map_[joint_name] = model_->GetJoint(joint_name);
      std::cout << "Joint # " << i << " - " << joint_name << std::endl;
    }

    q_.resize(joints.size());
    dq_.resize(joints.size());
    tau_.resize(joints.size());
    q_ctrl_.resize(joints.size());
    dq_ctrl_.resize(joints.size());
    tau_ctrl_.resize(joints.size());
    for (unsigned int i = 0; i < joint_names_.size(); i++){
      unsigned int index = 0;
      std::string n = joint_names_[i];
      q_[i] = joint_map_[n]->Position(index);
      dq_[i] = joint_map_[n]->GetVelocity(index);
      tau_[i] = joint_map_[n]->GetForce(index);
    }

        for (uint i = 0; i < 4; i++)
    {
      q_ctrl_[3*i] = q_[3*i];
      q_ctrl_[3*i+1] = -q_[3*i+1];
      q_ctrl_[3*i+2] = -q_[3*i+2];
      dq_ctrl_[3*i] = dq_[3*i];
      dq_ctrl_[3*i+1] = -dq_[3*i+1];
      dq_ctrl_[3*i+2] = -dq_[3*i+2];
      tau_ctrl_[3*i] = tau_[3*i];
      tau_ctrl_[3*i+1] = -tau_[3*i+1];
      tau_ctrl_[3*i+2] = -tau_[3*i+2];
    }

    // Enable currentloop response limit of the motors
    use_currentloop_response_ = true;
    // Enable TN curve limit of the motors
    use_TNcurve_motormodel_ = true;
    // Disable force contact sensors of the robot
    use_force_contact_sensor_ = true;

    simparam_->FirstRun();

    // Initialize LCMHandler
    lcmhandler_ = new LCMHandler();

    // Matching gazebo update frequency with control program frequency
    frequency_counter_=0; 

    // count leg for transfering contact forces to body coordnate
    foot_counter_ = 0;

  } // LeggedPlugin::Load

  // Called by the world update start event
  void LeggedPlugin::OnUpdate()
  {
    // Matching gazebo update frequency with control program frequency
    frequency_counter_++;

    GetJointStates();

    if(frequency_counter_<2)
    {
      SetJointCom();
      return;
    }
    
    // Send data of robot state by sharedmemory to contorl program 
    SendSMData();

    // Received and set joint command of robot from control program 
    SetJointCom();

    // Get contact force from foot contact sensor
    if(use_force_contact_sensor_) {
      GetContactForce4();
    }
    
    // Send simulator states by lcm
    lcmhandler_->SendSimData(lcm_sim_handler_);

    // Receive ros topic
    node_executor_->ReceiveTopic();

    // Apply force to the links of robot if command is received 
    ApplyForce();

    frequency_counter_=0; 

  }

  void LeggedPlugin::GetJointStates(){
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
      unsigned int index = 0;
      std::string n = joint_names_[i];
      q_[i] = joint_map_[n]->Position(index);
      dq_[i] = joint_map_[n]->GetVelocity(index);
      tau_[i] = joint_map_[n]->GetForce(index);
      
    }

    for (uint i = 0; i < 4; i++)
    {
      q_ctrl_[3*i] = q_[3*i];
      q_ctrl_[3*i+1] = -q_[3*i+1];
      q_ctrl_[3*i+2] = -q_[3*i+2];
      dq_ctrl_[3*i] = dq_[3*i];
      dq_ctrl_[3*i+1] = -dq_[3*i+1];
      dq_ctrl_[3*i+2] = -dq_[3*i+2];
      tau_ctrl_[3*i] = tau_[3*i];
      tau_ctrl_[3*i+1] = -tau_[3*i+1];
      tau_ctrl_[3*i+2] = -tau_[3*i+2];
    }
  }

  void LeggedPlugin::SendSMData()
  {

    // Read IMU data
    simToRobot.vectorNav.quat[3] = imu_sensor_->Orientation().W();
    simToRobot.vectorNav.quat[0] = imu_sensor_->Orientation().X();
    simToRobot.vectorNav.quat[1] = imu_sensor_->Orientation().Y();
    simToRobot.vectorNav.quat[2] = imu_sensor_->Orientation().Z();

    simToRobot.vectorNav.quat.normalize();

    simToRobot.vectorNav.gyro.x() = imu_sensor_->AngularVelocity()[0];
    simToRobot.vectorNav.gyro.y() = imu_sensor_->AngularVelocity()[1];
    simToRobot.vectorNav.gyro.z() = imu_sensor_->AngularVelocity()[2];

    simToRobot.vectorNav.accelerometer.x() = imu_sensor_->LinearAcceleration()[0];
    simToRobot.vectorNav.accelerometer.y() = imu_sensor_->LinearAcceleration()[1];
    simToRobot.vectorNav.accelerometer.z() = imu_sensor_->LinearAcceleration()[2];
    
    /************to  controller by spiDate************/
    for (uint i = 0; i < 4; i++)
    {
      simToRobot.spiData.q_abad[kleg_map[i]] = q_ctrl_[3 * i + 0];
      simToRobot.spiData.q_hip[kleg_map[i]] = q_ctrl_[3 * i + 1];
      simToRobot.spiData.q_knee[kleg_map[i]] = q_ctrl_[3 * i + 2];
      simToRobot.spiData.qd_abad[kleg_map[i]] = dq_ctrl_[3 * i + 0];
      simToRobot.spiData.qd_hip[kleg_map[i]] = dq_ctrl_[3 * i + 1];
      simToRobot.spiData.qd_knee[kleg_map[i]] = dq_ctrl_[3 * i + 2];
      simToRobot.spiData.tau_abad[kleg_map[i]] = tau_ctrl_[3 * i + 0];
      simToRobot.spiData.tau_hip[kleg_map[i]] = tau_ctrl_[3 * i + 1];
      simToRobot.spiData.tau_knee[kleg_map[i]] = tau_ctrl_[3 * i + 2];
    }

    for (uint i = 0; i < 4; i++)
    {
      for (uint j = 0; j<3; j++)
      {
        lcm_sim_handler_.q[kleg_map[i]*3+j]=q_ctrl_[i*3+j];
        lcm_sim_handler_.qd[kleg_map[i]*3+j]=dq_ctrl_[i*3+j];
        lcm_sim_handler_.tau[kleg_map[i]*3+j]=tau_ctrl_[i*3+j];
      }
    }

    // Read body states
    ignition::math::Pose3d base_pose;
    ignition::math::Vector3d base_vel;
    ignition::math::Vector3d base_omega;

    for (auto &it : model_->GetLinks()) {
      if (it->GetName() == "base_link") {
        base_pose = it->WorldPose();
        base_vel = it->WorldLinearVel();
        base_omega = it->WorldAngularVel();
      }
    }
  
    simToRobot.cheaterState.position.x() = base_pose.Pos()[0];
    simToRobot.cheaterState.position.y() = base_pose.Pos()[1];
    simToRobot.cheaterState.position.z() = base_pose.Pos()[2];
    simToRobot.cheaterState.orientation[0] = base_pose.Rot().W();
    simToRobot.cheaterState.orientation[1] = base_pose.Rot().X();
    simToRobot.cheaterState.orientation[2] = base_pose.Rot().Y();
    simToRobot.cheaterState.orientation[3] = base_pose.Rot().Z();

    Eigen::Quaterniond q(base_pose.Rot().W(),
                        base_pose.Rot().X(),
                        base_pose.Rot().Y(),
                        base_pose.Rot().Z());

    q_body_.w()=base_pose.Rot().W();
    q_body_.x()=base_pose.Rot().X(),
    q_body_.y()=base_pose.Rot().Y(),
    q_body_.z()=base_pose.Rot().Z(),
              

    q_body_.normalize();


    lcm_sim_handler_.quat[0] = base_pose.Rot().W();
    lcm_sim_handler_.quat[1] = base_pose.Rot().X();
    lcm_sim_handler_.quat[2] = base_pose.Rot().Y();
    lcm_sim_handler_.quat[3] = base_pose.Rot().Z();

    
    Eigen::Vector3d eulerAngle=q_body_.matrix().eulerAngles(2,1,0);
    lcm_sim_handler_.rpy[0]=eulerAngle[2];
    lcm_sim_handler_.rpy[1]=eulerAngle[1];
    lcm_sim_handler_.rpy[2]=eulerAngle[0];    

    simToRobot.cheaterState.vBody.x() = base_vel[0];
    simToRobot.cheaterState.vBody.y() = base_vel[1];
    simToRobot.cheaterState.vBody.z() = base_vel[2];
    simToRobot.cheaterState.omegaBody.x() = base_omega[0];
    simToRobot.cheaterState.omegaBody.y() = base_omega[1];
    simToRobot.cheaterState.omegaBody.z() = base_omega[2];

    simToRobot.cheaterState.vBody = q.conjugate() * simToRobot.cheaterState.vBody;
    simToRobot.cheaterState.omegaBody = q.conjugate() * simToRobot.cheaterState.omegaBody;

    for (uint i = 0; i < 3; i++) {
      lcm_sim_handler_.p[i]= base_pose.Pos()[i];
      lcm_sim_handler_.v[i]= base_vel[i];
      lcm_sim_handler_.vb[i] = simToRobot.cheaterState.vBody[i];
      lcm_sim_handler_.omega[i] = base_omega[i];
      lcm_sim_handler_.omegab[i] = simToRobot.cheaterState.omegaBody[i];

    }
    
    // Read gamepad command if gamepad command is received by lcmhandler
    if(lcmhandler_->HasEvent()) {
      simToRobot.gamepadCommand = lcmhandler_->ReceiveGPC();
      simparam_->LcmHasEvent();
    }

    // Send data of robot state by sharedmemory to contorl program 
    simparam_->SendSMData(simToRobot);

  }

  void LeggedPlugin::SetJointCom()
  {
    // Receive joint command by sharedmemory from contorl program 
    SpiCommand cmd = simparam_->ReceiveSMData();

    // Calculate motor torque by joint command 
    for (int i = 0; i < 4; i++) {
      unsigned int index = 0;
      double abad_effort = cmd.kp_abad[i] * (cmd.q_des_abad[i] - q_ctrl_[kleg_map[i]*3]) + cmd.kd_abad[i] * (cmd.qd_des_abad[i] - dq_ctrl_[kleg_map[i]*3]) + cmd.tau_abad_ff[i];
      double hip_effort = -(cmd.kp_hip[i] * (cmd.q_des_hip[i] - q_ctrl_[kleg_map[i]*3+1]) + cmd.kd_hip[i] * (cmd.qd_des_hip[i] - dq_ctrl_[kleg_map[i]*3+1]) + cmd.tau_hip_ff[i]);
      double knee_effort = -(cmd.kp_knee[i] * (cmd.q_des_knee[i] - q_ctrl_[kleg_map[i]*3+2]) + cmd.kd_knee[i] * (cmd.qd_des_knee[i] - dq_ctrl_[kleg_map[i]*3+2]) + cmd.tau_knee_ff[i]);


      if(use_TNcurve_motormodel_){
          abad_effort = motor_.GetTorque(abad_effort, dq_[kleg_map[i]*3]);
          hip_effort = motor_.GetTorque(hip_effort, dq_[kleg_map[i]*3+1]);
          knee_effort = motor_.GetTorque(knee_effort, dq_[kleg_map[i]*3+2]);
        }
        
      if(use_currentloop_response_){
          abad_effort = motor_.CerrentLoopResponse(abad_effort,dq_[kleg_map[i]*3],kleg_map[i]*3);
          hip_effort = motor_.CerrentLoopResponse(hip_effort,dq_[kleg_map[i]*3+1],kleg_map[i]*3+1);
          knee_effort = motor_.CerrentLoopResponse(knee_effort,dq_[kleg_map[i]*3+2],kleg_map[i]*3+2);
       }

      joint_map_[joint_names_[kleg_map[i]*3]]->SetForce(index, abad_effort);
      joint_map_[joint_names_[kleg_map[i]*3+1]]->SetForce(index, hip_effort);
      joint_map_[joint_names_[kleg_map[i]*3+2]]->SetForce(index, knee_effort);    
    }

  }

  void LeggedPlugin::GetContactForce4()
  {
    _contact_force contact_force_fl_;
    _contact_force contact_force_fr_;
    _contact_force contact_force_hl_;
    _contact_force contact_force_hr_;

    contact_force_fl_= GetContactForce(contact_sensor_fl_->Contacts());
    contact_force_fr_= GetContactForce(contact_sensor_fr_->Contacts());
    contact_force_hl_= GetContactForce(contact_sensor_hl_->Contacts());
    contact_force_hr_= GetContactForce(contact_sensor_hr_->Contacts());

    // Transfer contact forces to body coordnate
    for(unsigned int i=0; i<3; i++) {
      lcm_sim_handler_.f_foot[i] = LeggedPlugin::forceToBody(contact_force_fl_,model_)[i];
      lcm_sim_handler_.f_foot[i+3] = LeggedPlugin::forceToBody(contact_force_fr_,model_)[i];
      lcm_sim_handler_.f_foot[i+6] = LeggedPlugin::forceToBody(contact_force_hl_,model_)[i];
      lcm_sim_handler_.f_foot[i+9] = LeggedPlugin::forceToBody(contact_force_hr_,model_)[i];
      }

  }

  Eigen::Vector3d LeggedPlugin::forceToBody(_contact_force &_contact_force, physics::ModelPtr model_)
  {
    // Transfer contact forces to body coordnate
    Eigen::Vector3d force;
    if(foot_counter_ >3)
    foot_counter_ =0;
      if(_contact_force.parent_name=="") {
        force << 0,0,0;
      }
      else{
        auto parent_link = model_->GetLink(_contact_force.parent_name);
        ignition::math::Pose3d parent_pose;
        parent_pose = parent_link->WorldPose();
        Eigen::Quaterniond q(
                           parent_pose.Rot().W(),                                                 
                           parent_pose.Rot().X(),
                           parent_pose.Rot().Y(),
                           parent_pose.Rot().Z()                      
                           );
        q.normalize();

        
        Eigen::Matrix3d rotationMatrix=q.toRotationMatrix();
        Eigen::Matrix3d rotationMatrixW=q_body_.toRotationMatrix();

        force = rotationMatrixW.transpose()* rotationMatrix* _contact_force.force;
      }

      foot_counter_ ++;
      return force;
  }

  void LeggedPlugin::ForceHandler(const cyberdog_msg::msg::ApplyForce::SharedPtr msg)
  {
    // Handle ApplyForce topic message 
    apply_force_.name = msg -> link_name;
    apply_force_.time = msg -> time;
    for(int i=0;i<3;i++) {
      apply_force_.force[i]=msg -> force[i];
      apply_force_.rel_pos[i]=msg -> rel_pos[i];
    }
  }

  void LeggedPlugin::ApplyForce()
  {
    // Apply force to the links
    if(apply_force_.time > 0) {
      apply_force_.time = apply_force_.time - 0.001;
      gazebo::physics::LinkPtr link=model_->GetLink(apply_force_.name);
      link->AddForceAtRelativePosition(apply_force_.force,apply_force_.rel_pos);
    }
  }

}
