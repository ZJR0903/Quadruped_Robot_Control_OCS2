
//
// Created by qiayuan on 1/24/22.
//

#include "unitree_hw/hardware_interface.h"

namespace legged
{
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  udp.Init("192.168.123.8", 8801);
  udp.Sendmsg((char*)msg);
  udp.Recvmsg((char*)buf,12800);

  udp1.Init("192.168.123.6", 8801);
  udp1.Sendmsg((char*)msg1);
  udp1.Recvmsg((char*)buf1,12800);
  
  udp.Sendmsg((char*)msg);
  udp1.Sendmsg((char*)msg1);
  static int t;
  while ((t < 50) && !isAllValid(pos_zero))
  {
    udp.Sendmsg((char*)msg);
    udp.Recvmsg((char*)buf,12800);

    udp1.Sendmsg((char*)msg1);
    udp1.Recvmsg((char*)buf1,12800);

    t += 1;
    for(int i=0;i<6;i++){
      motor_zero.parseMsg(buf+78*i);
      motor_zero1.parseMsg(buf1+78*i);
      if (pos_zero[i] == 0)
        pos_zero[i] = motor_zero.Pos;
      if (pos_zero[i+6] == 0)
        pos_zero[i+6] = motor_zero1.Pos;
    }
  }

  direction_map.insert(map<string, vector<int>>::value_type("F", directionF));
  direction_map.insert(map<string, vector<int>>::value_type("B", directionB));

  // ros::shutdown();

  odom_sub_ = root_nh.subscribe("/imu/data", 1, &UnitreeHW::OdomCallBack, this);
  goal_sub_ = root_nh.subscribe("/move_base_simple/goal", 1, &UnitreeHW::GoalCallBack, this);
  foot_sensor_sub_ = root_nh.subscribe("/foot/sensor", 1, &UnitreeHW::FootSensorCallBack, this);
  
  listener_ = new tf::TransformListener(root_nh, ros::Duration(5.0), true);
  if (!QuadHW::init(root_nh, robot_hw_nh))
    return false;

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(low_cmd_);

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
  if (robot_type == "a1")
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  else if (robot_type == "aliengo")
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  else
  {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& period)
{
  // read imu 
  
  // udp_->Recv();
  // udp_->GetRecv(low_state_);

  udp.Recvmsg((char*)buf,12800);
  udp1.Recvmsg((char*)buf1,12800);

  for(int i=0;i<6;i++){
    motor.parseMsg(buf+78*i);
    motor1.parseMsg(buf1+78*i);
    last_msg_v[i] = motor.Pos;
    last_msg_v[i+6] = motor1.Pos;
    low_state_.motorState[i].q = 
      (1/9.1)*(last_msg_v[i] - pos_zero[i])*direction_map.find("F")->second[i] + pos_getdown[i];
    low_state_.motorState[i+6].q = 
      (1/9.1)*(last_msg_v[i+6] - pos_zero[i+6])*direction_map.find("B")->second[i] + pos_getdown[i];    
    CurrentPos[i]=(motor.Pos);
    CurrentPos[i+6]=(motor1.Pos);
    low_state_.motorState[i].tauEst=(9.1)*motor.T*direction_map.find("F")->second[i];
    low_state_.motorState[i+6].tauEst=(9.1)*motor1.T*direction_map.find("B")->second[i];
    low_state_.motorState[i].dq = (1/9.1)*motor.W*direction_map.find("F")->second[i];
    low_state_.motorState[i+6].dq = (1/9.1)*motor1.W*direction_map.find("B")->second[i];
  }

  for (int i = 0; i < 20; ++i)
  {
    joint_data_[i].pos_ = low_state_.motorState[i].q;
    joint_data_[i].vel_ = low_state_.motorState[i].dq;
    joint_data_[i].tau_ = low_state_.motorState[i].tauEst;   
  }

  // imu_data_.ori[0] = low_state_.imu.quaternion[1];
  // imu_data_.ori[1] = low_state_.imu.quaternion[2];
  // imu_data_.ori[2] = low_state_.imu.quaternion[3];
  // imu_data_.ori[3] = low_state_.imu.quaternion[0];
  // imu_data_.angular_vel[0] = low_state_.imu.gyroscope[0];
  // imu_data_.angular_vel[1] = low_state_.imu.gyroscope[1];
  // imu_data_.angular_vel[2] = low_state_.imu.gyroscope[2];
  // imu_data_.linear_acc[0] = low_state_.imu.accelerometer[0];
  // imu_data_.linear_acc[1] = low_state_.imu.accelerometer[1];
  // imu_data_.linear_acc[2] = low_state_.imu.accelerometer[2];

    imu_data_.ori[0] = yesenceIMU_.orientation.x;
    imu_data_.ori[1] = yesenceIMU_.orientation.y;
    imu_data_.ori[2] = yesenceIMU_.orientation.z;
    imu_data_.ori[3] = yesenceIMU_.orientation.w;
    imu_data_.angular_vel[0] = yesenceIMU_.angular_velocity.x;
    imu_data_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
    imu_data_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
    imu_data_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;
    imu_data_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
    imu_data_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;
    for (int i = 9; i < 9; ++i)
    {
      imu_data_.ori_cov[i] = yesenceIMU_.orientation_covariance[i];
      imu_data_.angular_vel_cov[i] = yesenceIMU_.angular_velocity_covariance[i];
      imu_data_.linear_acc_cov[i] = yesenceIMU_.linear_acceleration_covariance[i];
    }

  
  // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
  // {
  //   contact_state_[i] = low_state_.footForce[i] > contact_threshold_;
  // }

  double contact_check = 0.282;
  if (getLegLength(low_state_.motorState[1].q, low_state_.motorState[2].q) > contact_check) {fsm.FSM_FR.estimate_contact = true;}
  else fsm.FSM_FR.estimate_contact = false;
  if (getLegLength(low_state_.motorState[4].q, low_state_.motorState[5].q) > contact_check) {fsm.FSM_FL.estimate_contact = true;}
  else fsm.FSM_FL.estimate_contact = false;
  if (getLegLength(low_state_.motorState[7].q, low_state_.motorState[8].q) > contact_check) {fsm.FSM_BR.estimate_contact = true;}
  else fsm.FSM_BR.estimate_contact = false;
  if (getLegLength(low_state_.motorState[10].q, low_state_.motorState[11].q) > contact_check) {fsm.FSM_BL.estimate_contact = true;}
  else fsm.FSM_BL.estimate_contact = false;

  contact_state_[0] = fsm.FSM_FR.estimate_contact;
  contact_state_[1] = fsm.FSM_FL.estimate_contact;
  contact_state_[2] = fsm.FSM_BR.estimate_contact;
  contact_state_[3] = fsm.FSM_BL.estimate_contact;

  if (!is_active_)
  {
    cout << "===false===" << endl;
    contact_state_[0] = true;
    contact_state_[1] = true;
    contact_state_[2] = true;
    contact_state_[3] = true;
  }
  // for (int i=0;i<4;++i) contact_state_[i]=contact_state_cb_[i];
  cout << contact_state_[0] << " " << contact_state_[1] << " " << contact_state_[2] << " " << contact_state_[3] << endl;

  // softlimit

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybrid_joint_interface_.getNames();
  for (const auto& name : names)
  {
    HybridJointHandle handle = hybrid_joint_interface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(0.);
  }
}

void UnitreeHW::write(const ros::Time& time, const ros::Duration& period)
{
  // cout<<"write"<<endl;
  for (int i = 0; i < 20; ++i)
  {
    low_cmd_.motorCmd[i].q = joint_data_[i].pos_des_;
    low_cmd_.motorCmd[i].dq = joint_data_[i].vel_des_;
    low_cmd_.motorCmd[i].Kp = joint_data_[i].kp_;
    low_cmd_.motorCmd[i].Kd = joint_data_[i].kd_;
    low_cmd_.motorCmd[i].tau = joint_data_[i].ff_;
  }
  safety_->PositionLimit(low_cmd_);
  
  // udp_->SetSend(low_cmd_);
  // udp_->Send();

  map<string, vector<int>>::iterator it;  

  for (int i = 0; i < 6; ++i) 
  {
    /*
    1: mode 
    2: pos
    3: kp
    4: vel
    5: kd
    6: tau
    */
   // 1: mode
    msg[i*6] = 10;
    msg1[i*6] = 10;
    //2: pos
    msg[i*6+1] = 9.1*(low_cmd_.motorCmd[i].q - pos_getdown[i])*direction_map.find("F")->second[i] + pos_zero[i];
    msg1[i*6+1] = 9.1*(low_cmd_.motorCmd[i+6].q - pos_getdown[i])*direction_map.find("B")->second[i] + pos_zero[i+6];
    //3 : kp
    msg[i*6+2] = low_cmd_.motorCmd[i].Kp * 0.0167;
    msg1[i*6+2] = low_cmd_.motorCmd[i].Kp * 0.0167;
    //4: vel
    msg[i*6+3] = low_cmd_.motorCmd[i].dq*direction_map.find("F")->second[i]*9.1;
    msg1[i*6+3] = low_cmd_.motorCmd[i+6].dq*direction_map.find("B")->second[i]*9.1;
    //5 : kd
    msg[i*6+4] = low_cmd_.motorCmd[i].Kd;
    msg1[i*6+4] = low_cmd_.motorCmd[i+6].Kd;
    // 6: tau
    msg[i*6+5] = (1/9.1)*low_cmd_.motorCmd[i].tau*direction_map.find("F")->second[i];
    msg1[i*6+5] = (1/9.1)*low_cmd_.motorCmd[i+6].tau*direction_map.find("B")->second[i];
  }

  if (int(msg[0*6+2]) == 0)
  {
    static float deltpos[4]  = {-0.1, 0.1, -0.1, 0.1};
    static float step = 0.01;
    for(int i=0;i<4;i++) 
    {
      if(fabs(9.1*pos_getdown[i*3])>deltpos[i]) deltpos[i] += step;
    }
    // cout << "deltpos [2] "<< deltpos[2] << " " << deltpos[3] << endl;
    msg[0*6+1] = pos_zero[0] - deltpos[0]*(pos_getdown[0]>0?1:-1) ;
    msg[3*6+1] = pos_zero[3] - deltpos[1]*(pos_getdown[3]>0?1:-1) ;
    msg1[0*6+1] = pos_zero[0+6] + deltpos[2]*(pos_getdown[6]>0?1:-1);
    msg1[3*6+1] = pos_zero[3+6] + deltpos[3]*(pos_getdown[9]>0?1:-1);

    float q=3;
    float p=0.4;

    msg[0*6+2] = p;
    msg[0*6+4] = q;
    msg[3*6+2] = p;
    msg[3*6+4] = q;
    msg1[0*6+2] = p*2;
    msg1[0*6+4] = q;
    msg1[3*6+2] = p*2;
    msg1[3*6+4] = q;
  }

  udp.Sendmsg((char*)msg);
  udp1.Sendmsg((char*)msg1);
}

void UnitreeHW::smoothPos(vector<float>& msg, const vector<float>& last_msg)
{
  double max_pos_linear_acceleration_ = 0.01;
  if (msg[0] == 0) return;

  for (int i = 0; i < 12; ++i)
  {
    if(msg[i] > last_msg[i]) 
    {
      if (last_msg[i] + max_pos_linear_acceleration_ <= msg[i])
      {
        msg[i] = last_msg[i] + max_pos_linear_acceleration_;
      }
    }
    else if(msg[i] < last_msg[i]) 
    {
      if (last_msg[i] - max_pos_linear_acceleration_ >= msg[i])
      {
        msg[i] = last_msg[i] - max_pos_linear_acceleration_;
      }
    }
  }
}

bool UnitreeHW::setupJoints()
{
  for (const auto& joint : urdf_model_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("RF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint.first.find("LF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint.first.find("RH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint.first.find("LH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RL_;
    else
      continue;
    if (joint.first.find("HAA") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("HFE") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("KFE") != std::string::npos)
      joint_index = 2;
    else
      continue;

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                      &joint_data_[index].tau_);
    joint_state_interface_.registerHandle(state_handle);
    hybrid_joint_interface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                             &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                             &joint_data_[index].kd_, &joint_data_[index].ff_));
  }

  return true;
}

bool UnitreeHW::setupImu()
{
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
      imu_data_.linear_acc, imu_data_.linear_acc_cov));
  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh)
{
  nh.getParam("contact_threshold", contact_threshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    contact_sensor_interface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contact_state_[i]));
  return true;
}

}  // namespace legged