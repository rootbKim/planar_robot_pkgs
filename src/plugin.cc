#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <fstream>
#include <rbdl/rbdl.h>
#include <planar_robot_pkgs/Planar_CTCMsg.h>
#include <planar_robot_pkgs/Pub_torque_calc_Msg.h>
#include <planar_robot_pkgs/Pub_torque_calc_Msg2.h>

using namespace std;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef struct Joint_torque
{
  double torque;
} JOINT;

// ***************행렬 선언*********************//
double th[2] = {0, };

//*************** Trajectroy Variables**************//    
unsigned int cnt = 0;

//*************** 확인용 변수*********************//
unsigned int f_cnt = 0; //주기 확인용 
unsigned int c_cnt = 0; // 시행횟수

namespace gazebo
{
  class PLANAR_ROBOT_plugin : public ModelPlugin
  {
    // ************* Model variables ****************//
    physics::ModelPtr model;

    physics::LinkPtr UPPER_BODY_LINK;
    physics::LinkPtr LOWER_BODY_LINK;

    physics::JointPtr UPPER_BODY_JOINT;
    physics::JointPtr LOWER_BODY_JOINT;

    VectorXd target_tor = VectorXd::Zero(2);

    // ************* Joint space variables ****************// 기본이 열벡터임에 주의할것!
    VectorXd pre_target_joint_pos = VectorXd::Zero(2);
    VectorXd target_joint_pos = VectorXd::Zero(2);  // IK pos
    VectorXd target_joint_vel = VectorXd::Zero(2);
    VectorXd target_joint_acc = VectorXd::Zero(2);
    VectorXd actual_joint_pos = VectorXd::Zero(2);
    VectorXd pre_actual_joint_pos = VectorXd::Zero(2);
    VectorXd actual_joint_vel = VectorXd::Zero(2);
    VectorXd actual_joint_acc = VectorXd::Zero(2);
    VectorXd joint_pos_err = VectorXd::Zero(2);
    VectorXd joint_vel_err = VectorXd::Zero(2);
    VectorXd Kp_q = VectorXd::Zero(2);
    VectorXd Kd_q = VectorXd::Zero(2);
    VectorXd cancle_delay = VectorXd::Zero(2);
    VectorXd error = VectorXd::Zero(2);
    VectorXd goal_joint_pos = VectorXd::Zero(2); // FK pos

    // ************* Cartesian space variables ****************//
    VectorXd target_EP_pos = VectorXd::Zero(2);
    VectorXd target_EP_vel = VectorXd::Zero(2);
    VectorXd target_EP_acc = VectorXd::Zero(2);
    VectorXd actual_EP_pos = VectorXd::Zero(2);
    VectorXd actual_EP_vel = VectorXd::Zero(2);
    VectorXd actual_EP_acc = VectorXd::Zero(2);
    VectorXd init_EP_pos = VectorXd::Zero(2);
    VectorXd goal_EP_pos = VectorXd::Zero(2);
    VectorXd EP_pos_err = VectorXd::Zero(2);

    // *************Time variables ****************//
    common::Time last_update_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time = 0;
    common::Time current_time;

    // *************IMU sensor variables ****************//
    math::Pose base_info;
    sensors::SensorPtr Sensor;

    // ************* ROS Communication ****************//
    ros::NodeHandle n;
	  // ************* publisher ************************//
    ros::Publisher P_Times;
    ros::Publisher P_ros_msg;
    
    ros::Publisher P_actual_U_J;
    ros::Publisher P_actual_L_J;

    ros::Publisher P_goal_U_J;
    ros::Publisher P_goal_L_J;

    // ************ msg ***************** //
    std_msgs::Float64 m_Times;
    
    std_msgs::Float64MultiArray m_ros_msg;
    
    std_msgs::Float64 m_actual_U_J;
    std_msgs::Float64 m_actual_L_J;
        
    std_msgs::Float64 m_goal_U_J;
    std_msgs::Float64 m_goal_L_J;
    	
    VectorXd TmpData = VectorXd::Zero(50);
    ros::Subscriber server_sub1;
    ros::Subscriber server_sub2;

    ros::Publisher pub_tmp;
    planar_robot_pkgs::Pub_torque_calc_Msg pub_msg;
    ros::Subscriber sub_tmp;

    int start_flag = 0;

    // ************* Structure variables ****************//
    JOINT* joint;
    JOINT* old_joint;

    enum ControlMode
    {
      IDLE = 0,
      GRAVITY_CONTROL,
      CTC_CONTROL,
      CTC_CONTROL_POS,
      CTC_CONTROL_CONT_POS,
    };
    
    enum ControlMode CONTROL_MODE;

    // ************* Functions ****************//
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    void UpdateAlgorithm();
    void RBDL_INIT();
    void GetLinks();
    void GetJoints();
    void InitROSPubSetting();
    void EncoderRead();
    void jointController();
    void ROSMsgPublish();
    void Callback1(const std_msgs::Int32Ptr &msg);
    void torque_calc(const planar_robot_pkgs::Pub_torque_calc_Msg2::ConstPtr &msg);

    VectorXd FK(VectorXd joint_pos_HS);
    VectorXd IK(VectorXd EP_pos);

    FILE* tmpdata0=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdata0.txt","w");
    FILE* tmpdata1=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdata1.txt","w");

    void Print(void); //Print function

  };
  GZ_REGISTER_MODEL_PLUGIN(PLANAR_ROBOT_plugin); //model plugin 등록함수
}

void gazebo::PLANAR_ROBOT_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) //처음키면 한번 실행되는 함수
{
  this->model = _model;
  GetLinks();
  GetJoints();
  InitROSPubSetting();
  
  joint = new JOINT[2];
  old_joint = new JOINT[2];
  CONTROL_MODE = IDLE;

  this->last_update_time = this->model->GetWorld()->GetSimTime();
  this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PLANAR_ROBOT_plugin::UpdateAlgorithm, this));
  std::cout << "Load..." << std::endl;
}

void gazebo::PLANAR_ROBOT_plugin::UpdateAlgorithm() // 여러번 실행되는 함수
{
  //************************** Time ********************************//
  current_time = this->model->GetWorld()->GetSimTime();
  dt = current_time.Double() - this->last_update_time.Double();

  f_cnt++;

  EncoderRead(); //FK 푸는것도 포함.

  jointController();
  ROSMsgPublish();
  Print();

  this->last_update_time = current_time;

}

void gazebo::PLANAR_ROBOT_plugin::GetLinks() 
{
  //LINK DEFINITION
  this->UPPER_BODY_LINK = this->model->GetLink("UPPER_BODY_LINK");
  this->LOWER_BODY_LINK = this->model->GetLink("LOWER_BODY_LINK");
}

void gazebo::PLANAR_ROBOT_plugin::GetJoints()
{
  //JOINT DEFINITION
  this->UPPER_BODY_JOINT = this->model->GetJoint("UPPER_BODY_JOINT");
  this->LOWER_BODY_JOINT = this->model->GetJoint("LOWER_BODY_JOINT");
}

void gazebo::PLANAR_ROBOT_plugin::InitROSPubSetting()
{
  //************************ROS Msg Setting*********************************//
  P_Times = n.advertise<std_msgs::Float64>("times", 1);
  
  P_actual_U_J = n.advertise<std_msgs::Float64>("actual_U_J", 10);
  P_actual_L_J = n.advertise<std_msgs::Float64>("actual_L_J", 10);

  P_goal_U_J = n.advertise<std_msgs::Float64>("goal_U_J", 10);
  P_goal_L_J = n.advertise<std_msgs::Float64>("goal_L_J", 10);
  
  P_ros_msg = n.advertise<std_msgs::Float64MultiArray>("TmpData", 50); // topicname, queue_size = 50

  pub_tmp = n.advertise<planar_robot_pkgs::Pub_torque_calc_Msg>("torque_calc_msg", 10);

  m_ros_msg.data.resize(50);
  server_sub1 = n.subscribe("Ctrl_mode", 1, &gazebo::PLANAR_ROBOT_plugin::Callback1, this);
  sub_tmp = n.subscribe("torque_calc_msg2", 100, &gazebo::PLANAR_ROBOT_plugin::torque_calc, this);
}

void gazebo::PLANAR_ROBOT_plugin::EncoderRead()
{
  //************************** Encoder ********************************//
  actual_joint_pos[0] = this->UPPER_BODY_JOINT->GetAngle(0).Radian();
  actual_joint_pos[1] = this->LOWER_BODY_JOINT->GetAngle(0).Radian();

  for (int i = 0; i < 2; i++)
  {
    th[i] = double(actual_joint_pos[i]);
  }

  pub_msg.th[0] = th[0];
  pub_msg.th[1] = th[1];
  pub_msg.mode = CONTROL_MODE;
  pub_msg.cnt = f_cnt;
  pub_tmp.publish(pub_msg);
}

void gazebo::PLANAR_ROBOT_plugin::jointController()
{
  //* Torque Limit 감속기 정격토크참조함.
  for (unsigned int i = 0; i < 2; ++i)
  {
    if (joint[i].torque >= 8560*3)
    {
      joint[i].torque = 8560*3;
    }
    else if (joint[i].torque <= -8560*3)
    {
      joint[i].torque = -8560*3;
    }
  }

  //* Applying torques
  this->UPPER_BODY_JOINT->SetForce(0, joint[0].torque); //SUBO3.target_tor[0]);
  this->LOWER_BODY_JOINT->SetForce(0, joint[1].torque); //SUBO3.target_tor[1]);
}

void gazebo::PLANAR_ROBOT_plugin::Callback1(const std_msgs::Int32Ptr &msg)
{
  cnt=0;       
  
  if (msg->data == 0) //button-6
  {
    cnt = 0;
    CONTROL_MODE = IDLE;
  }       
  else if (msg->data == 1) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GRAVITY_CONTROL;
  }
  else if (msg->data == 2) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL;
  }
  else if (msg->data == 3) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL_POS;
  }
  else if (msg->data == 4) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL_CONT_POS;
  }
  else
  {
    cnt = 0;
    CONTROL_MODE = IDLE;
  }
}

void gazebo::PLANAR_ROBOT_plugin::torque_calc(const planar_robot_pkgs::Pub_torque_calc_Msg2::ConstPtr &msg)
{
  joint[0].torque = msg->torque[0];
  joint[1].torque = msg->torque[1];
}

void gazebo::PLANAR_ROBOT_plugin::Print() // 한 싸이클 돌때마다 데이터 플로팅
{
  if(CONTROL_MODE == CTC_CONTROL_CONT_POS)
  {
    fprintf(tmpdata0, "%f, %f\n", th[0], th[1]);
    fprintf(tmpdata1, "%f, %f\n", joint[0].torque, joint[1].torque);
  }
}

void gazebo::PLANAR_ROBOT_plugin::ROSMsgPublish()
{
  //********************* Data_plot - FT SENSOR***************************//
  TmpData[0] = joint[0].torque;  // L_PELVIS_YAW_JOINT
  TmpData[1] = joint[1].torque;  // L_PELVIS_ROLL_JOINT

  //****************** msg에 데이터 저장 *****************//
  for (unsigned int i = 0; i < 1; ++i)
  {
      m_ros_msg.data[i] = TmpData[i];
  }  
  
  P_ros_msg.publish(m_ros_msg);
}