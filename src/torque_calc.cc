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
#include <thread>

//************** define **************//
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
#define dt 0.001

//************** namespace **************//
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

JOINT* joint;
JOINT* old_joint;

//************** publish variable **************//
ros::Publisher pub_tmp;
planar_robot_pkgs::Pub_torque_calc_Msg2 pub_msg;

int mode;   // control mode
unsigned int main_cnt = 0;  // check loop
unsigned int loop_cnt = 0;  // check loop
int start_flag = 0;
//************** control variable **************//
double Theo_th[2] = {0, };  // init_traj target angle
double th[2] = {0, };
double prev_th[2] = {0, };

//************** torque timer variable **************//
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
double chg_step_time = 0;
double chg_cnt_time = 0;
unsigned int chg_cnt = 0;

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

//************** PD control variable **************//
VectorXd Kp_q = VectorXd::Zero(2);
VectorXd Kd_q = VectorXd::Zero(2);

//************** Data save variable **************//
FILE* tmpdatA0=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdatA0.txt","w");
FILE* tmpdatA1=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdatA1.txt","w");
FILE* tmpdatA2=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdatA2.txt","w");
FILE* tmpdatA3=fopen("/home/jiyong/catkin_ws/src/planar_robot_pkgs/MATLAB/tmpdatA3.txt","w");

//************** Function **************//
void msgCallback(const planar_robot_pkgs::Pub_torque_calc_Msg::ConstPtr& msg);
void msgCallback2(const planar_robot_pkgs::Planar_CTCMsg::ConstPtr &msg);
void torque_calc();
void RBDL_INIT();
void Init_Pos_Traj();
void Gravity_Cont();
void Calc_Feedback_Pos_();
void Calc_CTC_Torque_();
void Calc_Feedback_Pos();
void Calc_CTC_Torque();
void CTC_Control();
void CTC_Control_Pos();
void CTC_Control_Cont_Pos();

//************** Main Function **************//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_subscriber");

    joint = new JOINT[2];
    old_joint = new JOINT[2];
    
    RBDL_INIT();

    ros::NodeHandle nh;

    pub_tmp = nh.advertise<planar_robot_pkgs::Pub_torque_calc_Msg2>("torque_calc_msg2", 10);

    ros::Subscriber sub_tmp = nh.subscribe("torque_calc_msg",10,msgCallback);
    ros::Subscriber sub_tmp2 = nh.subscribe("Planar_CTC_msg", 100, msgCallback2);

    std::thread worker(torque_calc);

    ros::spin();     
    worker.join();

    return 0;
}

void msgCallback(const planar_robot_pkgs::Pub_torque_calc_Msg::ConstPtr& msg)
{
    Q(0) = msg->th[0];
    Q(1) = msg->th[1];

    th[0] = Q(0);
    th[1] = Q(1);

    mode = msg->mode;
    main_cnt = msg->cnt;

    QDot = (Q - prevQ) / dt;
    QDDot = (QDot - prevQDot) / dt;

    prevQ = Q;
    prevQDot = QDot;

    Calc_Feedback_Pos_();
    // Calc_Feedback_Pos();
}

void msgCallback2(const planar_robot_pkgs::Planar_CTCMsg::ConstPtr &msg)
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

void torque_calc()
{
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        cout << "=====================================================================" << endl;
        cout << "main cnt: " << main_cnt << " main time: " << main_cnt * 0.001 << endl;
        cout << "loop cnt: " << loop_cnt << " loop time: " << loop_cnt * 0.0005 << endl << endl;

        loop_cnt++;

        if(mode == 0)
        {
            Init_Pos_Traj();
        }
        else if(mode == 1)
        {
            Gravity_Cont();
        }
        else if(mode == 2)
        {
            CTC_Control();
        }
        else if(mode == 3)
        {
            CTC_Control_Pos();
        }
        else if(mode == 4)
        {
            CTC_Control_Cont_Pos();
        }
        
        // pintf function
        if(mode == 4)
        {
            fprintf(tmpdatA0, "%f, %f\n", Des_X(0), Des_X(1));
            fprintf(tmpdatA1, "%f, %f\n", Foot_Pos(0), Foot_Pos(1));
            fprintf(tmpdatA2, "%f, %f\n", torque_CTC(0), torque_CTC(1));
            fprintf(tmpdatA3, "%f, %f\n", joint[0].torque, joint[1].torque);
        }

        pub_tmp.publish(pub_msg);
        loop_rate.sleep();
    }
}

void RBDL_INIT()
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

//************** Torque Calculate Function **************//
void Calc_Feedback_Pos_()
{
    Math::Vector3d Pos;

    // F.K.
    double c1, c12, s1 ,s12;
    c1 = cos(Q(0)); c12 = cos(Q(0)+Q(1)); s1 = sin(Q(0)); s12 = sin(Q(0)+Q(1));
    Pos(0) = L0*s1 + L1*s12;
    Pos(1) = -(L0*c1 + L1*c12);

    Jacobian << L0*c1+L1*c12, L1*c12, L0*s1+L1*s12, L1*s12;
    Inv_Jacobian << L1*s12, -L1*c12, -L0*s1-L1*s12, L0*c1+L1*c12;
    Inv_Jacobian /= ((L0*c1+L1*c12)*(L1*s12) - (L1*c12)*(L0*s1+L1*s12));

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0 ; j< 2; j++)
        {
        Jacobian_dot(i,j) = (Jacobian(i,j) - prev_Jacobian(i,j)) / 0.001;
        prev_Jacobian(i,j) = Jacobian(i,j);
        }
        QDot_(i) = (th[i] - prev_th[i]) / 0.001;
        prev_th[i] = th[i];
    }

    // Current Pos & Pos_dot
    Foot_Pos(0) = Pos(0);
    Foot_Pos(1) = Pos(1);
    Foot_Pos_dot = Jacobian*QDot_;
}

void Calc_CTC_Torque_()
{
    double c1, c12, s1 ,s12;
    c1 = cos(Q(0)); c12 = cos(Q(0)+Q(1)); s1 = sin(Q(0)); s12 = sin(Q(0)+Q(1));

    Kp << 50000, 50000;
    Kv << 100, 100;

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

    NE_Tau(0) = 2*M1*L0*Lc1*QDot_(0)*QDot_(1)*sin(Q(1)) + M1*L0*Lc1*QDot_(1)*QDot_(1)*sin(Q(1)) - M0*g*Lc0*sin(Q(0)) - M1*g*L0*sin(Q(0)) - M1*g*Lc1*sin(Q(0)+Q(1));
    NE_Tau(1) = M1*L0*Lc1*QDot_(0)*QDot_(0)*sin(Q(1)) - M1*g*Lc1*sin(Q(0)+Q(1));

    torque_CTC = I_Matrix * q_CTC + NE_Tau;
}

void Calc_Feedback_Pos()
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
        Jacobian_dot(i,j) = (Jacobian(i,j) - prev_Jacobian(i,j)) / 0.001;
        prev_Jacobian(i,j) = Jacobian(i,j);
        }
        QDot_tmp(i) = QDot(i);
    }

    // Current Pos & Pos_dot
    Foot_Pos(0) = Pos(1);
    Foot_Pos(1) = Pos(2);
    Foot_Pos_dot = Jacobian*QDot_tmp;
}

void Calc_CTC_Torque()
{
    VectorNd QDot_tmp;
    QDot_tmp = VectorNd::Zero(2);

    if(start_flag == 0)
    {
        Kp << 5000, 5000;
        Kv << 100, 100;
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
}

//************** Pose Generation **************//
void Init_Pos_Traj()
{
    Kp_q << 50, 50;
    Kd_q << 5, 5;

    step_time = 2; //주기설정 (초) 변수
    cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    cnt++;
    
    double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

    if(cnt_time <= step_time)
    {
        Theo_th[0] = 30*Init_trajectory*deg2rad;
        Theo_th[1] = -60*Init_trajectory*deg2rad;
    }

    QDot = (Q - prevQ) / dt;
    QDDot = (QDot - prevQDot) / dt;

    prevQ = Q;
    prevQDot = QDot;

    ///////////////토크 입력////////////////
    for (int i = 0; i < 2; i++)
    {
        joint[i].torque = Kp_q[i]*(Theo_th[i] - Q(i)) + Kd_q[i] * (0 - QDot(i)); // 기본 PV제어 코드
        old_joint[i].torque = joint[i].torque;

        pub_msg.torque[i] = joint[i].torque;
    }
}

void Gravity_Cont()
{
    step_time = 1; //주기설정 (초) 변수
    cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    cnt++;

    QDot = (Q - prevQ) / dt;
    QDDot = (QDot - prevQDot) / dt;

    prevQ = Q;
    prevQDot = QDot;

    // InverseDynamics(*rbdl_model, Q, VectorNd::Zero(3), VectorNd::Zero(3), Tau, NULL);
    joint[0].torque = - M0*g*Lc0*sin(Q(0)) - M1*g*L0*sin(Q(0)) - M1*g*Lc1*sin(Q(0)+Q(1));
    joint[1].torque = - M1*g*Lc1*sin(Q(0)+Q(1));

    ///////////////토크 입력////////////////
    for (int i = 0; i < 2; i++)
    {
        pub_msg.torque[i] = joint[i].torque;

        old_joint[i].torque = joint[i].torque;
    }
}

void CTC_Control()
{
    step_time = 1; //주기설정 (초) 변수
    cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    cnt++;
    
    double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
    double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

    // Target Pos, Pos Dot, Pos DDot
    Des_X(0) = 0;  Des_X(1) = -0.43;
    Des_XDot(0) = 0;  Des_XDot(1) = 0;
    Des_XDDot(0) = 0;  Des_XDDot(1) = 0;

    // Calc_CTC_Torque_();    // calculate the CTC torque
    Calc_CTC_Torque();    // calculate the CTC torque

    if(cnt_time <= step_time)
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
            pub_msg.torque[i] = joint[i].torque;
        }
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = torque_CTC(i);
            pub_msg.torque[i] = joint[i].torque;

            old_joint[i].torque = joint[i].torque;
        }
    }
}

void CTC_Control_Pos()
{
    step_time = 1; //주기설정 (초) 변수
    cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    cnt++;
    
    double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
    double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

    // Target Pos, Pos Dot, Pos DDot
    if(start_flag == 0)
    {
        Des_X(0) = 0;  Des_X(1) = -0.43;
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

    Calc_CTC_Torque();    // calculate the CTC torque

    if(cnt_time <= step_time)
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
            pub_msg.torque[i] = joint[i].torque;
        }
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = torque_CTC(i);
            pub_msg.torque[i] = joint[i].torque;

            old_joint[i].torque = joint[i].torque;
        }
    }
}

void CTC_Control_Cont_Pos()
{
    step_time = 1; //주기설정 (초) 변수
    cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    cnt++;
    
    double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
    double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

    // Target Pos, Pos Dot, Pos DDot
    if(start_flag == 0)
    {
        Des_X(0) = 0;  Des_X(1) = -0.43;
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
        
        New_Des_X << 0, -0.47;
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
        
        New_Des_X << 0, -0.35;
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
            start_flag = 1;
            chg_cnt = 0;
        }
    }

    Calc_CTC_Torque_();    // calculate the CTC torque

    if(cnt_time <= step_time)
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = old_joint[i].torque*old_trajectory + torque_CTC(i)*new_trajectory;
            pub_msg.torque[i] = joint[i].torque;
        }
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            joint[i].torque = torque_CTC(i);
            pub_msg.torque[i] = joint[i].torque;

            old_joint[i].torque = joint[i].torque;
        }
    }
}