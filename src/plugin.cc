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

// ************* 단위변환 ************************//
#define PI 3.14159265358979
#define M2R 2*PI/4096
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323

#define L0	0.250
#define L1	0.250
#define Lc0	0.125
#define Lc1	0.125
#define M0  5
#define M1  5
#define I0  0.027083
#define I1  0.027083
#define g  -9.81

// ***************행렬 선언*********************//
double Theo_th[2] = {0, };
double th[2] = {0, };
double prev_th[2] = {0, };
//*************** Trajectroy Variables**************//    
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
double chg_step_time = 0;
double chg_cnt_time = 0;
unsigned int chg_cnt = 0;

//*************** 확인용 변수*********************//
unsigned int f_cnt = 0; //주기 확인용 
unsigned int c_cnt = 0; // 시행횟수

//*************** RBDL variable ***************//
VectorNd Kp, Kv;
//Left_Leg
Model* rbdl_model = NULL;//make model but emty
unsigned int upper_id, lower_id, body_end_id;
Body upper, lower, body_end;
Joint joint_upper, joint_lower, joint_body_end;
Math::Matrix3d upperI, lowerI, body_endI;
//declare Q -> Q is theta
VectorNd Q, QDot, QDDot, prevQ, prevQDot, Tau, Foot_Pos, Foot_Pos_dot, Des_X, Des_XDot, Des_XDDot, torque_CTC, Old_Des_X, Old_Des_XDot, Old_Des_XDDot, New_Des_X, New_Des_XDot, New_Des_XDDot;
MatrixNd Jacobian, prev_Jacobian, Jacobian_dot, Inv_Jacobian;

VectorNd QDot_;

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
    void msgCallback(const planar_robot_pkgs::Planar_CTCMsg::ConstPtr &msg);

    void PostureGeneration();
    void Calc_Feedback_Pos();
    void Calc_CTC_Torque();
    void Calc_Feedback_Pos_();
    void Calc_CTC_Torque_();
    void Init_Pos_Traj();
    void Gravity_Cont();
    void CTC_Control();
    void CTC_Control_Pos();
    void CTC_Control_Cont_Pos();

    VectorXd FK(VectorXd joint_pos_HS);
    VectorXd IK(VectorXd EP_pos);

    void Print(void); //Print function

  };
  GZ_REGISTER_MODEL_PLUGIN(PLANAR_ROBOT_plugin); //model plugin 등록함수
}

void gazebo::PLANAR_ROBOT_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) //처음키면 한번 실행되는 함수
{
  RBDL_INIT();

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

void gazebo::PLANAR_ROBOT_plugin::RBDL_INIT()
{
  //********************RBDL********************//
  rbdl_check_api_version(RBDL_API_VERSION);//check the rdbl version

  Kp = VectorNd::Zero(6);
  Kv = VectorNd::Zero(6);

  //RBDL_MODEL
  rbdl_model = new Model();//declare Model
  rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity

  upperI = Math::Matrix3d(0.027083,0,0, 0,0.027083,0, 0,0,0.002083);
  upper = Body(5, Math::Vector3d(0, 0, -0.125), upperI);
  joint_upper = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  upper_id = rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), joint_upper, upper);

  lowerI = Math::Matrix3d(0.027083,0,0, 0,0.027083,0, 0,0,0.002083);
  lower = Body(5, Math::Vector3d(0, 0, -0.125), lowerI);
  joint_lower = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  lower_id = rbdl_model->Model::AddBody(upper_id, Xtrans(Math::Vector3d(0, 0, -0.25)), joint_lower, lower);

  body_endI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
  body_end = Body(0, Math::Vector3d(0, 0, 0), body_endI);
  joint_body_end = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
  body_end_id = rbdl_model->Model::AddBody(lower_id, Xtrans(Math::Vector3d(0, 0, -0.25)), joint_body_end, body_end);

    //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	Q = VectorNd::Zero(rbdl_model->dof_count);
	QDot = VectorNd::Zero(rbdl_model->dof_count);
	QDDot = VectorNd::Zero(rbdl_model->dof_count);
	prevQ = VectorNd::Zero(rbdl_model->dof_count);
	prevQDot = VectorNd::Zero(rbdl_model->dof_count);
  Tau = VectorNd::Zero(rbdl_model->dof_count);
  Foot_Pos = VectorNd::Zero(2);
  Foot_Pos_dot = VectorNd::Zero(2);
  Jacobian = MatrixNd::Zero(2,2);
  prev_Jacobian = MatrixNd::Zero(2,2);
  Jacobian_dot = MatrixNd::Zero(2,2);
  Inv_Jacobian = MatrixNd::Zero(2,2);
  Des_X = VectorNd::Zero(2);
  Des_XDot = VectorNd::Zero(2);
  Des_XDDot = VectorNd::Zero(2);
  torque_CTC = VectorNd::Zero(2);
  Old_Des_X = VectorNd::Zero(2);
  Old_Des_XDot = VectorNd::Zero(2);
  Old_Des_XDDot = VectorNd::Zero(2);
  New_Des_X = VectorNd::Zero(2);
  New_Des_XDot = VectorNd::Zero(2);
  New_Des_XDDot = VectorNd::Zero(2);

  QDot_ = VectorNd::Zero(2);

  //init Q
  Q(0) = 0, prevQ(0) = 0, prevQDot(0) = 0;
  Q(1) = 0, prevQ(1) = 0, prevQDot(1) = 0;
  Q(2) = 0, prevQ(2) = 0, prevQDot(2) = 0;
}

void gazebo::PLANAR_ROBOT_plugin::UpdateAlgorithm() // 여러번 실행되는 함수
{
  //************************** Time ********************************//
  current_time = this->model->GetWorld()->GetSimTime();
  dt = current_time.Double() - this->last_update_time.Double();

  f_cnt++;
  if(f_cnt >= 1)
  {
    EncoderRead(); //FK 푸는것도 포함.
  }

  PostureGeneration(); // PostureGeneration 하위에 Trajectory 하위에 IK푸는것 포함.

  if(f_cnt >= 1)
  {
    jointController();
    ROSMsgPublish();
    Print();
    f_cnt = 0;
  }

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
  m_ros_msg.data.resize(50);
  server_sub1 = n.subscribe("Ctrl_mode", 1, &gazebo::PLANAR_ROBOT_plugin::Callback1, this);
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

void gazebo::PLANAR_ROBOT_plugin::msgCallback(const planar_robot_pkgs::Planar_CTCMsg::ConstPtr &msg)
{
  start_flag = msg->TF;
  if(start_flag == 1)
  {
    New_Des_X(0) = msg->Des_X_y;New_Des_X(1) = msg->Des_X_z;
    New_Des_XDot(0) = msg->Des_XDot_y;New_Des_XDot(1) = msg->Des_XDot_z;
    New_Des_XDDot(0) = msg->Des_XDDot_y;New_Des_XDDot(1) = msg->Des_XDDot_z;

    Kp(0) = msg->Kp0;
    Kp(1) = msg->Kp1;
    Kv(0) = msg->Kv0;
    Kv(1) = msg->Kv1;
  }
}

void gazebo::PLANAR_ROBOT_plugin::Calc_Feedback_Pos_()
{
  Math::Vector3d Pos;

  // F.K.
  double c1, c12, s1 ,s12;
  c1 = cos(th[0]); c12 = cos(th[0]+th[1]); s1 = sin(th[0]); s12 = sin(th[0]+th[1]);
  Pos(0) = L0*s1 + L1*s12;
  Pos(1) = -(L0*c1 + L1*c12);

  Jacobian << L0*c1+L1*c12, L1*c12, L0*s1+L1*s12, L1*s12;
  Inv_Jacobian << L1*s12, -L1*c12, -L0*s1-L1*s12, L0*c1+L1*c12;
  Inv_Jacobian /= ((L0*c1+L1*c12)*(L1*s12) - (L1*c12)*(L0*s1+L1*s12));

  for(int i = 0; i < 2; i++)
  {
    for(int j = 0 ; j< 2; j++)
    {
      Jacobian_dot(i,j) = (Jacobian(i,j) - prev_Jacobian(i,j)) / dt;
      prev_Jacobian(i,j) = Jacobian(i,j);
    }
    QDot_(i) = (th[i] - prev_th[i]) / dt;
    prev_th[i] = th[i];
  }

  // Current Pos & Pos_dot
  Foot_Pos(0) = Pos(0);
  Foot_Pos(1) = Pos(1);
  Foot_Pos_dot = Jacobian*QDot_;
}

void gazebo::PLANAR_ROBOT_plugin::Calc_CTC_Torque_()
{
  double c1, c12, s1 ,s12;
  c1 = cos(th[0]); c12 = cos(th[0]+th[1]); s1 = sin(th[0]); s12 = sin(th[0]+th[1]);

  Kp << 1000, 1000;
  Kv << 50, 50;

  MatrixNd J_0, J_1;
  J_0 = MatrixNd::Zero(3,2);  J_0(0,0) = Lc0*c1; J_0(1,0) = Lc0*s1;
  J_1 = MatrixNd::Zero(3,2);  J_1(0,0) = L0*c1 + Lc1*c12; J_1(0,1) = Lc1*c12; J_1(1,0) = L0*s1+Lc1*s12; J_1(1,1) = Lc1*s12;

  VectorNd X_CTC;
  X_CTC = VectorNd::Zero(2);

  VectorNd q_CTC;
  q_CTC = VectorNd::Zero(2);

  VectorNd NE_Tau;
  NE_Tau = VectorNd::Zero(2);

  MatrixNd I_Matrix, I_Matrix_tmp;
  I_Matrix = MatrixNd::Zero(2,2);
  I_Matrix_tmp = MatrixNd::Zero(2,2);

  for(int i = 0; i < 2; i++)
  {
    X_CTC(i) = Des_XDDot(i) + Kp(i) * (Des_X(i) - Foot_Pos(i)) + Kv(i) * (Des_XDot(i) - Foot_Pos_dot(i));
  }

  q_CTC = Inv_Jacobian * (X_CTC - Jacobian_dot*QDot_);

  I_Matrix_tmp << I0+I1, I1, I1, I1;
  I_Matrix = M0*J_0.transpose()*J_0 + M1*J_1.transpose()*J_1 + I_Matrix_tmp;

  // I_Matrix(0,0) = M0*sqrt(Lc0) + M1*(sqrt(L0) + sqrt(Lc1) + 2*L0*Lc1*cos(th[1])) + I0 + I1;
  // I_Matrix(0,1) = M1*(sqrt(Lc1) + L0*Lc1*cos(th[1])) + I1;
  // I_Matrix(1,0) = M1*(sqrt(Lc1) + L0*Lc1*cos(th[1])) + I1;
  // I_Matrix(1,1) = M1*sqrt(Lc1) + I1;

  // NE_Tau(0) = -2*M1*L0*Lc1*QDot_(0)*QDot_(1)*sin(th[1]) - M1*L0*Lc1*sqrt(QDot_(1))*sin(th[1]) - M0*g*Lc0*sin(th[0]) - M1*g*Lc0*sin(th[0]) - M1*g*Lc1*sin(th[0]+th[1]);
  // NE_Tau(1) = M1*L0*Lc1*sqrt(QDot_(0))*sin(th[1]) - M1*g*Lc1*sin(th[0]+th[1]);
  
  NE_Tau(0) = 2*M1*L0*Lc1*QDot_(0)*QDot_(1)*sin(th[1]) + M1*L0*Lc1*QDot_(1)*QDot_(1)*sin(th[1]) - M0*g*Lc0*sin(th[0]) - M1*g*L0*sin(th[0]) - M1*g*Lc1*sin(th[0]+th[1]);
  NE_Tau(1) = M1*L0*Lc1*QDot_(0)*QDot_(0)*sin(th[1]) - M1*g*Lc1*sin(th[0]+th[1]);

  torque_CTC = I_Matrix * q_CTC + NE_Tau;

  cout << "===================" << endl;
  cout << torque_CTC << endl << endl;
}

void gazebo::PLANAR_ROBOT_plugin::Calc_Feedback_Pos()
{
  Math::Vector3d Pos;
  VectorNd QDot_tmp;
  MatrixNd Jacobian8;

  QDot_tmp = VectorNd::Zero(2);
  Jacobian8 = MatrixNd::Zero(3,rbdl_model->dof_count);
   
  // Get the End Effector's Aixs
  Pos = CalcBodyToBaseCoordinates(*rbdl_model, Q, body_end_id, Math::Vector3d(0, 0, 0), true);
  // Get the Jacobian
  CalcPointJacobian(*rbdl_model, Q, body_end_id, Math::Vector3d(0, 0, 0), Jacobian8, true);
  // Get the Jacobian for 6 by 6
  for(int i = 0; i < 2; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      Jacobian(i, j) = Jacobian8(i+1, j);
    }
  }

  // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian
  Inv_Jacobian = Jacobian.inverse();
  // Calculate the Jacobian dot
  for(int i = 0; i < 2; i++)
  {
    for(int j = 0 ; j< 2; j++)
    {
      Jacobian_dot(i,j) = (Jacobian(i,j) - prev_Jacobian(i,j)) / dt;
      prev_Jacobian(i,j) = Jacobian(i,j);
    }
    QDot_tmp(i) = QDot(i);
  }

  // Current Pos & Pos_dot
  Foot_Pos(0) = Pos(1);
  Foot_Pos(1) = Pos(2);
  Foot_Pos_dot = Jacobian*QDot_tmp;
}

void gazebo::PLANAR_ROBOT_plugin::Calc_CTC_Torque()
{
  VectorNd QDot_tmp;
  QDot_tmp = VectorNd::Zero(2);

  if(start_flag == 0)
  {
    Kp << 1000, 1000;
    Kv << 50, 50;
  }

  VectorNd X_CTC;
  X_CTC = VectorNd::Zero(2);

  VectorNd q_CTC;
  q_CTC = VectorNd::Zero(2);

  VectorNd NE_Tau;
  NE_Tau = VectorNd::Zero(2);

  MatrixNd I_Matrix_tmp, I_Matrix;
  I_Matrix = MatrixNd::Zero(2,2);
  I_Matrix_tmp = MatrixNd::Zero(3,3);

  for(int i = 0; i < 2; i++)
  {
    X_CTC(i) = Des_XDDot(i) + Kp(i) * (Des_X(i) - Foot_Pos(i)) + Kv(i) * (Des_XDot(i) - Foot_Pos_dot(i));
    QDot_tmp(i) = QDot(i);
  }

  q_CTC = Inv_Jacobian * (X_CTC - Jacobian_dot*QDot_tmp);

  NonlinearEffects(*rbdl_model, Q, QDot, Tau, NULL);
  CompositeRigidBodyAlgorithm(*rbdl_model, Q, I_Matrix_tmp, true);

  for(int i = 0; i < 2; i++)
  {
    NE_Tau(i) = Tau(i);
    for(int j = 0; j < 2; j++)
    {
      I_Matrix(i,j) = I_Matrix_tmp(i,j);
    }
  }

  torque_CTC = I_Matrix * q_CTC + NE_Tau;

  cout << torque_CTC << endl << endl;
}

void gazebo::PLANAR_ROBOT_plugin::PostureGeneration()
{
  int test_cnt = 0;
  // cout << dt << endl;
  if (CONTROL_MODE == IDLE)
  {
    //std::cout << "Mode is Init_Pos_Mode" << std::endl;
    Init_Pos_Traj();
  }
  else if (CONTROL_MODE == GRAVITY_CONTROL) 
  {
	  test_cnt++;
    Gravity_Cont();
  }
  else if (CONTROL_MODE == CTC_CONTROL) 
  {
	  test_cnt++;
    CTC_Control();
  }
  else if (CONTROL_MODE == CTC_CONTROL_POS) 
  {
	  test_cnt++;
    CTC_Control_Pos();
  }
  else if (CONTROL_MODE == CTC_CONTROL_CONT_POS) 
  {
  test_cnt++;
    CTC_Control_Cont_Pos();
  }
}

void gazebo::PLANAR_ROBOT_plugin::Init_Pos_Traj()
{
  Kp_q << 500, 500;
  Kd_q << 5, 5;

  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double periodic_function_sin = sin(2*PI/step_time*cnt_time);
  double periodic_function_cos = cos(2*PI/step_time*cnt_time);
  double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  if(cnt_time <= step_time)
  {
    Theo_th[0] = 30*deg2rad;
    Theo_th[1] = -60*deg2rad;
  }
  
  for (int i = 0; i < 2; i++)
  {
    actual_joint_vel[i] = (actual_joint_pos[i] - pre_actual_joint_pos[i]) / dt;
    pre_actual_joint_pos[i] = actual_joint_pos[i];
  }

  ///////////////토크 입력////////////////
  for (int i = 0; i < 2; i++)
  {
    joint[i].torque = Kp_q[i]*(Theo_th[i] - actual_joint_pos[i]) + Kd_q[i] * (0 - actual_joint_vel[i]); // 기본 PV제어 코드
  }

  Q(0) = actual_joint_pos[0];
  Q(1) = actual_joint_pos[1];

  QDot = (Q - prevQ) / dt;
  QDDot = (QDot - prevQDot) / dt;

  prevQ = Q;
  prevQDot = QDot;
}

void gazebo::PLANAR_ROBOT_plugin::Gravity_Cont()
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  Q(0) = actual_joint_pos[0];
  Q(1) = actual_joint_pos[1];

  QDot = (Q - prevQ) / dt;
  QDDot = (QDot - prevQDot) / dt;

  prevQ = Q;
  prevQDot = QDot;

  InverseDynamics(*rbdl_model, Q, VectorNd::Zero(3), VectorNd::Zero(3), Tau, NULL);
  
  for (int i = 0; i < 2; i++)
  {
    joint[i].torque = Tau(i);
  }
}

void gazebo::PLANAR_ROBOT_plugin::CTC_Control()
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  Q(0) = actual_joint_pos[0];
  Q(1) = actual_joint_pos[1];

  QDot = (Q - prevQ) / dt;
  QDDot = (QDot - prevQDot) / dt;

  prevQ = Q;
  prevQDot = QDot;

  // Target Pos, Pos Dot, Pos DDot
  Des_X(0) = 0;  Des_X(1) = -0.43;
  Des_XDot(0) = 0;  Des_XDot(1) = 0;
  Des_XDDot(0) = 0;  Des_XDDot(1) = 0;

  Calc_Feedback_Pos_();  // calculate the feedback
  Calc_CTC_Torque_();    // calculate the CTC torque

  // Calc_Feedback_Pos();  // calculate the feedback
  // Calc_CTC_Torque();    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 2; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      joint[i].torque = torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
    }
  }
}

void gazebo::PLANAR_ROBOT_plugin::CTC_Control_Pos()
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  Q(0) = actual_joint_pos[0];
  Q(1) = actual_joint_pos[1];

  QDot = (Q - prevQ) / dt;
  QDDot = (QDot - prevQDot) / dt;

  prevQ = Q;
  prevQDot = QDot;

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    Des_X(0) = 0.1;  Des_X(1) = 0.1;
    Des_XDot(0) = 0;  Des_XDot(1) = 0;
    Des_XDDot(0) = 0;  Des_XDDot(1) = 0;

    Old_Des_X = Des_X; Old_Des_XDot = Des_XDot; Old_Des_XDDot = Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    if(chg_cnt_time <= chg_step_time)
    {
      Des_X = Old_Des_X + (New_Des_X - Old_Des_X)*change_trajectory;
      Des_XDot = Old_Des_XDot + (New_Des_XDot - Old_Des_XDot)*change_trajectory;
      Des_XDDot = Old_Des_XDDot + (New_Des_XDDot - Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      Des_X = New_Des_X; Des_XDot = New_Des_XDot; Des_XDDot = New_Des_XDDot;
      Old_Des_X = Des_X; Old_Des_XDot = Des_XDot; Old_Des_XDDot = Des_XDDot;
      start_flag = 2;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos();  // calculate the feedback
  Calc_CTC_Torque();    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 2; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
    }
  }
}

void gazebo::PLANAR_ROBOT_plugin::CTC_Control_Cont_Pos()
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  Q(0) = actual_joint_pos[0];
  Q(1) = actual_joint_pos[1];

  QDot = (Q - prevQ) / dt;
  QDDot = (QDot - prevQDot) / dt;

  prevQ = Q;
  prevQDot = QDot;

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    Des_X(0) = 0.1;  Des_X(1) = 0.1;
    Des_XDot(0) = 0;  Des_XDot(1) = 0;
    Des_XDDot(0) = 0;  Des_XDDot(1) = 0;

    Old_Des_X = Des_X; Old_Des_XDot = Des_XDot; Old_Des_XDDot = Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    
    New_Des_X << 0, 0.07;
    New_Des_XDot << 0, 0;
    New_Des_XDDot << 0, 0;

    if(chg_cnt_time <= chg_step_time)
    {
      Des_X = Old_Des_X + (New_Des_X - Old_Des_X)*change_trajectory;
      Des_XDot = Old_Des_XDot + (New_Des_XDot - Old_Des_XDot)*change_trajectory;
      Des_XDDot = Old_Des_XDDot + (New_Des_XDDot - Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      Des_X = New_Des_X; Des_XDot = New_Des_XDot; Des_XDDot = New_Des_XDDot;
      Old_Des_X = Des_X; Old_Des_XDot = Des_XDot; Old_Des_XDDot = Des_XDDot;
      start_flag = 2;
      chg_cnt = 0;
    }
  }
  else if(start_flag == 2)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    
    New_Des_X << 0, 0.07;
    New_Des_XDot << 0, 0;
    New_Des_XDDot << 0, 0;
    
    if(chg_cnt_time <= chg_step_time)
    {
      Des_X = Old_Des_X + (New_Des_X - Old_Des_X)*change_trajectory;
      Des_XDot = Old_Des_XDot + (New_Des_XDot - Old_Des_XDot)*change_trajectory;
      Des_XDDot = Old_Des_XDDot + (New_Des_XDDot - Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      Des_X = New_Des_X; Des_XDot = New_Des_XDot; Des_XDDot = New_Des_XDDot;
      Old_Des_X = Des_X; Old_Des_XDot = Des_XDot; Old_Des_XDDot = Des_XDDot;
      start_flag = 2;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos();  // calculate the feedback
  Calc_CTC_Torque();    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 2; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      joint[i].torque = torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
    }
  }
}

void gazebo::PLANAR_ROBOT_plugin::Print() // 한 싸이클 돌때마다 데이터 플로팅
{

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