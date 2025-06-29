// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

// 必须放在最前面，否则会出现编译错误
#include "pinocchio/fwd.hpp"

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

// 使用pinocchio lib
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
// 逆动力学库
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
// 惯性矩阵库
#include "pinocchio/algorithm/crba.hpp"
// 时钟库
#include <chrono>

// 多线程
#include <atomic>
#include <functional>
#include <thread>
// 线程锁
#include <mutex>

// qpOASES 求解器（需要安装 qpOASES 库）
#include <qpOASES.hpp>

#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

class PandaHocbfTestController : public controller_interface::MultiInterfaceController<
                                     hardware_interface::PositionJointInterface,
                                     hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  // 控制周期
  double dt;

  // 函数最大计算时间
  double max_time;

  // 创建7自由度eigen向量
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  // 状态结构体
  struct State {
    Vector7d q;
    Vector7d e_q;
    Vector7d qdot;
    Vector7d e_qdot;
    // Eigen::MatrixXd M_q;
    Vector7d e_torque;
    Vector7d qddot_norm;  // 关节名义加速度

    pinocchio::SE3 X;               // 执行器位姿(李群表示linear, angular)
    Eigen::Vector3d p;              // 末端位置
    Eigen::Vector3d pdot;           // 末端线速度
    Eigen::Vector3d Jdot_qdot;      // 非线性加速度项
    pinocchio::Data::Matrix6x J;    // 当前位姿的Jacobian矩阵panda_model.nv=7
    pinocchio::Data::Matrix3x J_p;  // 当前位姿的Jacobian矩阵panda_model.nv=7
    double h_0;                     // 安全距离
    Vector7d nonlinear;             // 非线性前馈项
  };

  // CBF变量结构体
  struct CBF_Params {
    // cbf开关
    bool CBF_switch;

    Vector7d f_vec;                 // 目标函数线性项，编码期望动力学
    Eigen::Matrix<double, 1, 7> A;  // 线性不等式约束矩阵
    Vector7d A_vec;                 // 线性不等式约束矩阵
    double b;                       // 线性不等式约束向量
    double b_value;                 // 线性不等式约束向量

    double alpha1;  // 阻尼参数
    double alpha2;  // 阻尼参数

    double h1;  // 严格安全边界
    // Eigen::RowVector3d C1;
    Eigen::Vector3d C1;
    double phi1;  // CBF约束函数，混合位置/速度与衰减系数alpha1
    double D1;

    Vector7d output_vec;  // 输出向量

    // 新增参数
    double T_pre;   // 安全时间阈值
    double t0_val;  // 初始时间
    double c_gain;  // 增益系数
    // 在类定义中添加成员变量
    double omega_;
    double t_d_;
    // 在类定义中添加成员变量
    double k;
    double k2;
  };

  // CBF参数
  CBF_Params cbf_params;

  // 控制参数结构体
  struct ControlParams {
    Eigen::DiagonalMatrix<double, 7> Kp_matrix;
    Eigen::DiagonalMatrix<double, 7> Kd_matrix;
  };

  // 控制参数
  ControlParams PD_control;

  // 当前状态
  State current_state;

  // 当前控制步数
  int current_step;

  // 关节轨迹点
  struct Joint_Waypoint {
    Vector7d position;      // 关节角度
    Vector7d velocity;      // 关节速度
    Vector7d acceleration;  // 关节加速度
    Vector7d torque;        // 关节力矩
    double timestamp;
    // int number;
  };

  // 末端执行器轨迹点
  struct Actuator_Waypoint {
    // Eigen::Matrix<double, 4, 4> X;    // 执行器位姿
    pinocchio::SE3 X;                 // 执行器位姿(李群表示linear, angular)
    Eigen::Vector3d p;                // 末端位置
    Eigen::Vector3d dp;               // 末端速度
    Eigen::Vector3d ddp;              // 末端加速度
    Eigen::Matrix<double, 3, 3> R;    // 执行器姿态
    Eigen::Matrix<double, 3, 3> dR;   // 执行器角速度
    Eigen::Matrix<double, 3, 3> ddR;  // 执行器角加速度
    double timestamp;
  };

  // 关节动作
  struct Joint_Action {
    Vector7d qstart;
    Vector7d qend;
    double Tf_duration;
    int num_waypoints;
    int method;
    std::vector<Joint_Waypoint> qDesired;
  };

  // 静止动作
  struct Stay_Action {
    Vector7d qstay;
    double Tf_duration;
    int num_waypoints;
    std::vector<Joint_Waypoint> qDesired;
  };

  // 末端执行器动作
  struct Actuator_Action {
    // Eigen::Matrix<double, 4, 4> Xstart;
    pinocchio::SE3 Xstart;
    // Eigen::Matrix<double, 4, 4> Xend;
    pinocchio::SE3 Xend;

    Eigen::Vector3d pstart;
    Eigen::Vector3d pend;

    Eigen::Matrix<double, 3, 3> Rstart;
    Eigen::Matrix<double, 3, 3> Rend;

    Actuator_Waypoint action_end;
    double Tf_duration;
    int num_waypoints;
    int method;
    std::vector<Actuator_Waypoint> X_desired;
    std::vector<Joint_Waypoint> qDesired;
  };

  // 初始化关节动作，将机械臂由初始位置（初始姿态）移动到action0初始位置
  Joint_Action action_init;
  // 保持姿态动作
  Stay_Action stay_init;
  // 关节动作0
  Joint_Action action0;
  // 保持姿态动作
  Stay_Action stay0;
  // 末端执行器动作1
  Actuator_Action action1;
  // 保持姿态动作
  Stay_Action stay1;
  // 末端执行器动作2
  Actuator_Action action2;
  // 保持姿态动作
  Stay_Action stay2;

  // 末端执行器动作圆
  Actuator_Action action_circle;

  // 全部轨迹
  std::vector<Joint_Waypoint> qDesired_total;

  // 关节速度限幅
  Vector7d joint_velocity_min_limit;
  Vector7d joint_velocity_max_limit;

  // 关节加速度限幅
  Vector7d joint_acc_min_limit;
  Vector7d joint_acc_max_limit;

  // 关节力矩限幅
  Vector7d joint_torque_min_limit;
  Vector7d joint_torque_max_limit;

  // 机器人模型
  pinocchio::Model panda_model;
  pinocchio::Data panda_data;
  std::string urdf_filename;

  // 末端执行器关节id, 固定关节只能用frame表示
  pinocchio::FrameIndex End_effector_joint_id;

  // jacobian矩阵阻尼参数
  double J_damping;

  // 障碍物位置
  Eigen::Vector3d obstacle_position;

  // 夹爪半径
  double obstacle_radius;

  // 关节轨迹插补函数
  std::vector<Joint_Waypoint> Joint_Trajectory(Vector7d& qstart,
                                               Vector7d& qend,
                                               double Tf_duration,
                                               int N,
                                               int method);

  // 笛卡尔轨迹插补函数
  std::vector<Actuator_Waypoint> Cartesian_Trajectory(Actuator_Action& action);

  // 圆形轨迹插补函数
  std::vector<Actuator_Waypoint> Plan_Circle_Trajectory(Actuator_Action& action,
                                                        Eigen::Vector3d obstacle_position);

  // 逆运动学迭代求解
  std::vector<Joint_Waypoint> Inverse_Kinematics(Actuator_Action& action, Vector7d& q_init);

  // 五次样条平滑函数
  double QuinticTimeScaling(double total_time, double current_time);

  // 静止姿态动作函数
  std::vector<Joint_Waypoint> Joint_Stay(Vector7d& qstay, int N, double Tf_duration);

  //  qpoases优化器线程
  void thread_task();

  // qpoases优化器结构体
  struct qpoases_opt {
    qpOASES::QProblem qp;
    qpOASES::Options options;
    Eigen::Matrix<double, 7, 7> H_qp;
    Eigen::Matrix<double, 7, 1> g_qp;
    Eigen::Matrix<double, 1, 7> A_qp;
    Eigen::Matrix<double, 7, 1> lb;
    Eigen::Matrix<double, 7, 1> ub;

    // QP 不等式约束为： A * τ ≥ b_constraint
    // qpOASES 要求 lbA ≤ A*τ ≤ ubA，设下界为 b_constraint，上界设为一个足够大的正数
    Eigen::Matrix<double, 1, 1> lbA;
    Eigen::Matrix<double, 1, 1> ubA;

    qpOASES::returnValue rval;
    Eigen::Matrix<double, 7, 1> tau_opt;
  };
  // 用于线程安全
  qpoases_opt qpoases_cbf;
  std::atomic<bool> qpoases_opt_startup;
  std::atomic<int> opt_step_last;
  std::atomic<int> opt_step_current;
  std::mutex opt_mutex_;

  // 滤波器参数
  Vector7d filter_new_mearsurement;
  double filter_alpha;

  // 判断是否进入球内

  bool is_in_sphere;
  bool left_sphere;
  bool stopped_at_outer_sphere;
  bool circle_trajectory_started_;
  Joint_Waypoint desire_stopped_joint_waypoint;

  //新增ros发布
  ros::Publisher debug_tau_pub_;
  ros::Subscriber gp_comp_sub_;
  std_msgs::Float64MultiArray gp_compensation_msg_;
  std::mutex gp_comp_mutex_;  // 线程安全锁

  void gpCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  // 新增函数
  //   double phi(double t, double T_pre, double t0_val, double c_gain);
  //   double phi_dot(double t, double T_pre, double t0_val, double c_gain);

 private:
  // 关节位置接口
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  // 关节力矩接口
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;

  // 当前时间
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
};

}  // namespace franka_example_controllers