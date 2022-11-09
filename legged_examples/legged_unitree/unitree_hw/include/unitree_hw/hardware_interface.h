
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/safety.h"

#include <legged_hw/hardware_interface.h>


#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "./udp.h"
#include "./unitree_motor.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/Int16MultiArray.h>

namespace legged
{
const std::vector<std::string> CONTACT_SENSOR_NAMES = { "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" };

enum State
{
  Early,
  Contact,
  Swing,
  Late
};

struct FSM_ELEMENT
{
  State state;
  bool time_contact;
  double delay_time;
  double begin_time;
  bool estimate_contact;
};

struct FSM
{
  FSM_ELEMENT FSM_FR, FSM_FL, FSM_BR, FSM_BL;
};

struct UnitreeMotorData
{
  double pos_, vel_, tau_;                   // state
  double pos_des_, vel_des_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class UnitreeHW : public QuadHW
{
private:
  FSM fsm;
  udp::Udp udp;
  udp::Udp udp1;
  motor::UnitreeMotor motor_zero;
  motor::UnitreeMotor motor_zero1;
  motor::UnitreeMotor motor;
  motor::UnitreeMotor motor1;

  char buf[78*6] = {0};
  char buf1[78*6] = {0};
  float CurrentPos[12]={0};
  float CurrentT[12]={0};
  float CurrentW[12]={0};
  vector<float> msg_v = vector<float>(12);
  vector<float> msg_dq = vector<float>(12);
  vector<float> last_msg_v = vector<float>(12);
  float msg[36] = {0};
  float msg1[36] = {0};
  float msg_zero[36] = {0};
  float msg_zero1[36] = {0};
  float pos_zero[12] = {0};
  // float pos_getdown_B[6] = {0, 1, -2.58, 0, 1, -2.58};
  // float pos_getdown_B[6] = {0-0.17, 1.2, -2.62+0.46-0.2, 0.17-0.07, 0.8, -2.6-0.58};

  const vector<int> directionF{ 1, -1, -1,  1,  1,  1};
  // const vector<int> directionFL{ 1,  1,  1};
  const vector<int> directionB{-1, -1, -1, -1,  1,  1};
  // const vector<int> directionBL{-1,  1,  1};
  map<string, vector<int>> direction_map;
  vector<std::pair<string, vector<int>>> direction;

public:
  // UnitreeHW() = default;
  UnitreeHW()
  {
    std::string string_name = "/home/zjr/record.txt";
    std::string string_name_1 = "/home/zjr/recordtime.txt";
    std::string string_name_2 = "/home/zjr/recordFR.txt";
    ofs.open(string_name, ios::trunc);
    ofs_1.open(string_name_1, ios::trunc);
    ofs_2.open(string_name_2, ios::trunc);

    fsm.FSM_FR.state = Contact;
    fsm.FSM_FL.state = Contact;
    fsm.FSM_BR.state = Contact;
    fsm.FSM_BL.state = Contact;

    fsm.FSM_FR.estimate_contact = true;
    fsm.FSM_FL.estimate_contact = true;
    fsm.FSM_BR.estimate_contact = true;
    fsm.FSM_BL.estimate_contact = true; 
  }
  ~UnitreeHW()
  {
    cout << "ofs.close();" << endl;
    ofs.close();
    ofs_1.close();
    ofs_2.close();
  }
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:

  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;

  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState low_state_{};
  UNITREE_LEGGED_SDK::LowCmd low_cmd_{};

  UnitreeMotorData joint_data_[20]{};
  UnitreeImuData imu_data_{};
  bool contact_state_[4]{};
  bool contact_state_cb_[4]{};
  int contact_threshold_{};

  private:
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  // foot
  ros::Subscriber foot_sensor_sub_;
  sensor_msgs::Imu yesenceIMU_;
  int FR, FL, BR, BL;
  bool is_active_;
  void OdomCallBack(const sensor_msgs::Imu::ConstPtr& odom)
  {
    yesenceIMU_ = *odom;
  }

  void GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& odom)
  {
    is_active_ = true;
  }

  void FootSensorCallBack(const std_msgs::Int16MultiArray::ConstPtr& msg)
  {
    for(int i=0;i<4;i++) contact_state_cb_[i] = msg->data[i];
  }

  inline bool isLegStance(double hip, double thigh, double calf, string leg)
  {
    if (leg == "FR")
    {
      // cout << "===isLegStance===" << endl;
      // if (hip > -0.09464 && hip < 0.13) 
        if (thigh > 0.8126 && thigh < 0.9271)
          if (calf > -1.8 && calf < -1.5)
            return true;

      if ((int)(contact_state_[0]) == 1 && FR == 0)
      {
        // cout << "FR : hip : " << hip << " thigh " << thigh << " calf " << calf << endl;
        // if (hip <= -0.09464 || hip >= 0.13) cout << "ERROR hip" << endl;
        // if (thigh <= 0.8126 || thigh >= 0.9171) cout << "ERROR thigh" << endl;
        // if (calf <= -1.8 || calf >= -1.5) cout << "ERROR calf" << endl;
      }
    }
    else if (leg == "FL")
    {
      // cout << "===isLegStance===" << endl;
      // if (hip > -0.09 && hip < 0.13) 
        if (thigh > 0.75 && thigh < 0.9)
          if (calf > -1.88 && calf < -1.65)
            return true;

      if ((int)(contact_state_[1]) == 1 && FL == 0)
      {
        // cout << "FL : hip : " << hip << " thigh " << thigh << " calf " << calf << endl;
        // if (hip <= -0.09 || hip >= 0.13) cout << "ERROR hip" << endl;
        // if (thigh <= 0.75 || thigh >= 0.9) cout << "ERROR thigh" << endl;
        // if (calf <= -1.88 || calf >= -1.65) cout << "ERROR calf" << endl;
      }
    }
    else if (leg == "BR")
    {
      // cout << "===isLegStance BR ===" << endl;
      // if (hip > -0.09 && hip < 0.13) 
        if (thigh > 0.75 && thigh < 0.96)
          if (calf > -1.88 && calf < -1.5)
            return true;

      if ((int)(contact_state_[2]) == 1 && BR == 0)
      {
        // cout << "BR : hip : " << hip << " thigh " << thigh << " calf " << calf << endl;
        // if (hip <= -0.09 || hip >= 0.13)  cout << "ERROR hip" << endl;
        // if (thigh <= 0.75 || thigh >= 0.6) cout << "ERROR thigh" << endl;
        // if (calf <= -1.88 || calf >= -1.5) cout << "ERROR calf" << endl;
      }
    }
    else if (leg == "BL")
    {
      // if (hip > -0.09 && hip < 0.13) 
        if (thigh > 0.75 && thigh < 0.9)
          if (calf > -1.88 && calf < -1.6)
            return true;

      if ((int)(contact_state_[3]) == 1 && BL == 0)
      {
        // cout << "BL : hip : " << hip << " thigh " << thigh << " calf " << calf << endl;
        // if (hip <= -0.09 || hip >= 0.13)  cout << "ERROR hip" << endl;
        // if (thigh <= 0.75 || thigh >= 0.9) cout << "ERROR thigh" << endl;
        // if (calf <= -1.88 || calf >= -1.6) cout << "ERROR calf" << endl;
      }
    }
    return false;
  }

  inline bool isBodyStance(const UNITREE_LEGGED_SDK::LowState& _lowState)
  {
    if (isSimilar(_lowState.motorState[0].q, _lowState.motorState[3].q, _lowState.motorState[6].q, _lowState.motorState[9].q))
      if (isSimilar(_lowState.motorState[1].q, _lowState.motorState[4].q, _lowState.motorState[7].q, _lowState.motorState[10].q))
        if (isSimilar(_lowState.motorState[2].q, _lowState.motorState[5].q, _lowState.motorState[8].q, _lowState.motorState[11].q))
          return true;
    return false;
  }

  inline bool isSimilar(float n1, float n2, float n3, float n4)
  {
    // cout << "===isSimilar===" << endl;
    vector<float> num_vec{fabs(n1), fabs(n2), fabs(n3), fabs(n4)};
    
    std::sort(num_vec.begin(), num_vec.end());
    
    if ((num_vec[3] - num_vec[2]) < 0.073 && (num_vec[2] - num_vec[1] < 0.145) && (num_vec[1] - num_vec[0] < 0.154))
      return true;
    // cout << fabs(n1) << " " << fabs(n2) << " " << fabs(n3) << " " << fabs(n4) << endl;
    // cout << "num_vec[3] - num_vec[2] " << num_vec[3] - num_vec[2] << endl;
    // cout << "num_vec[2] - num_vec[1] " << num_vec[2] - num_vec[1] << endl;
    // cout << "num_vec[1] - num_vec[0] " << num_vec[1] - num_vec[0] << endl;
    return false;
  }

  inline double getLegLength(double delta_1, double delta_2)
  {
    delta_1 = fabs(delta_1);
    delta_2 = fabs(delta_2);
    
    double length = 0.22 * (cos(delta_1) + cos(delta_2 - delta_1));
    
    if (length < 0.27)
    {
      // cout << "delta_1 " << delta_1 << " delta_2 " << delta_2 << endl;
      // cout << "cos(delta_1) " << cos(delta_1) << " cos(delta_2) " << cos(delta_2) << endl;
      // cout << "0.22*cos(delta_1) " << 0.22*cos(delta_1) << " 0.22*cos(delta_2) " << 0.22*cos(delta_2) << endl;
      // cout << "length " << length << endl;
    }
    return length;
  }

  std::ofstream ofs;
  std::ofstream ofs_1;
  std::ofstream ofs_2;

  void smoothPos(vector<float>& msg, const vector<float>& last_msg);
  bool isAllValid(float msg[])
  {
    for (int i = 0; i < 12; ++ i)
      if (fabs(msg[i]) < 0.01) 
        return false;
    return true;
  }

  /** Tf **/
  tf::TransformListener* listener_;
  // float pos_getdown[6] = {0, 1, -2.58, 0, 1, -2.58};
  float pos_getdown[12] = {-0.22, 1, -2.58, 0.22, 1, -2.58, -0.22, 1, -2.58, 0.22, 1, -2.58};
};

}  // namespace legged
