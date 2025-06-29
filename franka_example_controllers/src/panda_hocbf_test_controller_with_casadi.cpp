// // Copyright (c) 2017 Franka Emika GmbH
// // Use of this source code is governed by the Apache-2.0 license, see LICENSE
// #include <franka_example_controllers/panda_hocbf_test_controller.h>

// #include <cmath>

// #include <controller_interface/controller_base.h>
// #include <hardware_interface/hardware_interface.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.h>
// // #include <ros/ros.h>
// #include <rosconsole/macros_generated.h>
// #include <algorithm>
// #include <cmath>

// namespace franka_example_controllers {
// bool PandaHocbfTestController::init(hardware_interface::RobotHW* robot_hardware,
//                                     ros::NodeHandle& node_handle) {
//   // 获取关节力接口
//   effort_joint_interface_ = robot_hardware->get<hardware_interface::EffortJointInterface>();

//   if (effort_joint_interface_ == nullptr) {
//     ROS_ERROR("Error getting position or effort joint interface from hardware!");
//     return false;
//   }

//   // 获取关节名称
//   std::vector<std::string> joint_names;
//   if (!node_handle.getParam("joint_names", joint_names)) {
//     ROS_ERROR("Could not parse joint names");
//   }
//   if (joint_names.size() != 7) {
//     ROS_ERROR_STREAM("Wrong number of joint names, got " << joint_names.size()
//                                                          << " instead of 7 names!");
//     return false;
//   }

//   // 初始化关节句柄
//   effort_joint_handles_.resize(7);

//   for (std::size_t i = 0; i < 7; ++i) {
//     try {
//       // 获取关节句柄
//       effort_joint_handles_[i] = effort_joint_interface_->getHandle(joint_names[i]);
//     } catch (const hardware_interface::HardwareInterfaceException& e) {
//       ROS_ERROR_STREAM("Exception getting joint handles: " << e.what());
//       return false;
//     }
//   }

//   // 检查模型初始位置
//   //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//   //   for (std::size_t i = 0; i < q_start.size(); i++) {
//   //     if (std::abs(effort_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
//   //       ROS_ERROR_STREAM(
//   //           "JointPositionExampleController: Robot is not in the expected starting position

//   //           " "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
//   //       return false;
//   //     }
//   //   }

//   //   action_init.qend =
//   //       (PandaHocbfTestController::Vector7d() << 0, 0, 0, -1 * M_PI_2, 0, M_PI_2,
//   //       M_PI_4).finished();

//   std::array<double, 7> q_start;
//   //   std::array<double, 7> q_start_desired{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//   std::array<double, 7> q_start_desired{
//       {0, -M_PI_4 * 0.5, 0, -M_PI_2 - 0.5 * M_PI_4, 0, M_PI_2, M_PI_4}};
//   for (std::size_t i = 0; i < q_start.size(); i++) {
//     q_start[i] = effort_joint_handles_[i].getPosition();
//   }

//   // gazebo控制周期
//   dt = 0.001;

//   // 初始化步数
//   current_step = 0;

//   // 初始化逆运动学阻尼
//   J_damping = 1e-6;

//   // 滤波器参数
//   filter_alpha = 0.2;

//   //判断是否进入球内
//   is_in_sphere = false;
//   left_sphere = false;
//   stopped_at_outer_sphere = false;

//   // 开启避障
//   cbf_params.CBF_switch = true;
//   //   cbf_params.CBF_switch = false;

//   // 初始化cbf参数
//   cbf_params.H = 2 * casadi::DM::eye(7);
//   cbf_params.ubU = 0.8 * casadi::DM::vertcat({
//                              87.0, 87.0, 87.0, 87.0,  // 前4个关节扭矩上限
//                              12.0, 12.0, 12.0         // 后3个关节扭矩上限
//                          });
//   cbf_params.lbU = (-1) * cbf_params.ubU;
//   //   cbf_params.alpha1 = 5;
//   // 仿真参数
//   cbf_params.alpha1 = 0.5;
//   //   cbf_params.alpha1 = 0.5;
//   // 实验参数
//   //   cbf_params.alpha1 = 10;
//   //   cbf_params.alpha2 = 30;
//   //  final_cbf
//   // 仿真参数
//   cbf_params.alpha2 = 0.1;
//   //   cbf_params.alpha2 = 0.1;
//   // 实验参数
//   //   cbf_params.alpha2 = 3;

//   //实验参数
//   //   cbf_params.omega_ = 1.0;
//   //  仿真参数
//   cbf_params.omega_ = 0.1;
//   cbf_params.t_d_ = 20.001;

//   casadi_opt_startup = false;
//   opt_step_last = 0;
//   opt_step_current = 0;

//   // 初始化优化参数

//   casadi::Dict jitOptions;
//   jitOptions["flags"] = "-O3";
//   jitOptions["verbose"] = true;

//   casadiOptions["expand"] = true;
//   //   casadiOptions["error_on_fail"] = true;
//   casadiOptions["error_on_fail"] = false;
//   casadiOptions["print_time"] = false;

//   casadiOptions["jit"] = true;
//   casadiOptions["compiler"] = "shell";
//   casadiOptions["jit_options"] = jitOptions;

//   solverOptions["print_level"] = 0;
//   solverOptions["max_iter"] = 3000;
//   solverOptions["tol"] = 1e-8;
//   solverOptions["linear_solver"] = "ma57";

//   solverOptions["warm_start_init_point"] = "yes";
//   solverOptions["warm_start_bound_push"] = 1e-8;
//   solverOptions["warm_start_mult_bound_push"] = 1e-8;
//   solverOptions["mu_init"] = 1e-5;
//   solverOptions["bound_relax_factor"] = 1e-9;

//   // 障碍物位置
//   //   obstacle_position = Eigen::Vector3d(0.45, 0.0, 0.30);
//   //实验参数
//     // obstacle_position = Eigen::Vector3d(0.35, 0.0, 0.20);
//   // 仿真参数
//   obstacle_position = Eigen::Vector3d(0.60, 0.0, 0.20);

//   // 初始化夹爪半径
//   obstacle_radius = 0.03;

//   // 初始化控制参数
//   //   PD_control.Kp_matrix.diagonal() = 10 * PandaHocbfTestController::Vector7d::Ones() / dt;
//   PD_control.Kp_matrix.diagonal() = 300.0 * PandaHocbfTestController::Vector7d::Ones();
//   //   PD_control.Kd_matrix.diagonal() = 0.4 * PandaHocbfTestController::Vector7d::Ones() / dt;
//   PD_control.Kd_matrix.diagonal() = 30.0 * PandaHocbfTestController::Vector7d::Ones();

//   // 初始化关节限幅
//   joint_velocity_max_limit =
//       (PandaHocbfTestController::Vector7d() << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)
//           .finished();
//   joint_velocity_min_limit = (-1) * joint_velocity_max_limit;

//   joint_acc_max_limit =
//       (PandaHocbfTestController::Vector7d() << 15, 7.5, 10, 12.5, 15, 20, 20).finished();
//   joint_acc_min_limit = (-1) * joint_acc_max_limit;

//   joint_torque_max_limit =
//       (PandaHocbfTestController::Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();
//   joint_torque_min_limit = (-1) * joint_torque_max_limit;

//   // 绝对路径
//   urdf_filename = std::string(
//       "/home/robot/franka_ros_ws/src/franka_ros/franka_description/robots/panda/panda.urdf");
//   pinocchio::urdf::buildModel(urdf_filename, panda_model);
//   panda_data = pinocchio::Data(panda_model);

//   // 初始化动作
//   action_init.qstart = Eigen::Map<const PandaHocbfTestController::Vector7d>(q_start.data());
//   //   action_init.qend =
//   //       (PandaHocbfTestController::Vector7d() << 0, 0, 0, -1 * M_PI_2, 0, M_PI_2,
//   //       M_PI_4).finished();

//   action_init.qend = Eigen::Map<const PandaHocbfTestController::Vector7d>(q_start_desired.data());

//   action_init.Tf_duration = 3;
//   action_init.num_waypoints = (int)(action_init.Tf_duration / dt + 1);
//   action_init.method = 5;
//   action_init.qDesired =
//       Joint_Trajectory(action_init.qstart, action_init.qend, action_init.Tf_duration,
//                        action_init.num_waypoints, action_init.method);

//   // 初始化静止姿态动作
//   stay_init.qstay = action_init.qend;
//   stay_init.Tf_duration = 0.4;
//   stay_init.num_waypoints = (int)(stay_init.Tf_duration / dt + 1);
//   stay_init.qDesired = Joint_Stay(stay_init.qstay, stay_init.num_waypoints, stay_init.Tf_duration);

//   // 动作0
//   action0.qstart = stay_init.qstay;
//   action0.qend = (PandaHocbfTestController::Vector7d() << 0, -40 * M_PI / 180, 0, -130 * M_PI / 180,
//                   0, M_PI_2, M_PI_4)
//                      .finished();

//   //   action0.Tf_duration = 3.0;
//   action0.Tf_duration = 3;
//   action0.num_waypoints = (int)(action0.Tf_duration / dt + 1);
//   action0.method = 5;
//   action0.qDesired = Joint_Trajectory(action0.qstart, action0.qend, action0.Tf_duration,
//                                       action0.num_waypoints, action0.method);

//   // 动作0静止
//   stay0.qstay = action0.qend;
//   stay0.Tf_duration = 0.4;
//   stay0.num_waypoints = (int)(stay0.Tf_duration / dt + 1);
//   stay0.qDesired = Joint_Stay(stay0.qstay, stay0.num_waypoints, stay0.Tf_duration);

//   // 动作1
//   // 运动学解算
//   End_effector_joint_id = panda_model.getFrameId("panda_hand_tcp_joint");
//   pinocchio::framesForwardKinematics(panda_model, panda_data, stay0.qstay);

//   action1.Xstart = panda_data.oMf[End_effector_joint_id];
//   action1.Rstart = action1.Xstart.rotation();
//   action1.pstart = action1.Xstart.translation();

//   action1.Rend = action1.Rstart * pinocchio::exp3(Eigen::Vector3d(0, 0, (0 * M_PI / 180)));
//   action1.pend = action1.pstart + Eigen::Vector3d(0.0, 0.0, -0.3);
//   action1.Xend = pinocchio::SE3(action1.Rend, action1.pend);

//   //   action1.Tf_duration = 3.0;
//   action1.Tf_duration = 3;
//   action1.num_waypoints = (int)(action1.Tf_duration / dt + 1);
//   action1.method = 5;
//   action1.X_desired = Cartesian_Trajectory(action1);

//   //   逆运动学迭代
//   action1.qDesired = Inverse_Kinematics(action1, stay0.qstay);

//   // 动作1静止
//   stay1.qstay = action1.qDesired.back().position;
//   stay1.Tf_duration = 0.4;
//   stay1.num_waypoints = (int)(stay1.Tf_duration / dt + 1);
//   stay1.qDesired = Joint_Stay(stay1.qstay, stay1.num_waypoints, stay1.Tf_duration);

//   // 动作2
//   //   pinocchio::framesForwardKinematics(panda_model, panda_data, stay1.qstay);
//   action2.Xstart = action1.Xend;
//   action2.Rstart = action2.Xstart.rotation();
//   action2.pstart = action2.Xstart.translation();

//   action2.Rend = action2.Rstart * pinocchio::exp3(Eigen::Vector3d(0, 0, (0 * M_PI / 180)));
//   //   action2.pend = action2.pstart + Eigen::Vector3d(0.45, 0.0, 0.0);
//   action2.pend = action2.pstart + Eigen::Vector3d(0.30, 0.0, 0.0);
//   action2.Xend = pinocchio::SE3(action2.Rend, action2.pend);

//   action2.Tf_duration = 3;
//   action2.num_waypoints = (int)(action2.Tf_duration / dt + 1);
//   action2.method = 5;
//   action2.X_desired = Cartesian_Trajectory(action2);

//   //   逆运动学迭代
//   action2.qDesired = Inverse_Kinematics(action2, stay1.qstay);

//   // 动作2静止
//   stay2.qstay = action2.qDesired.back().position;
//   stay2.Tf_duration = 0.4;
//   stay2.num_waypoints = (int)(stay2.Tf_duration / dt + 1);
//   stay2.qDesired = Joint_Stay(stay2.qstay, stay2.num_waypoints, stay2.Tf_duration);

//   // 动作拼接
//   for (const auto& vec :
//        {action_init.qDesired, stay_init.qDesired, action0.qDesired, stay0.qDesired,
//         action1.qDesired, stay1.qDesired, action2.qDesired, stay2.qDesired}) {
//     qDesired_total.insert(qDesired_total.end(), vec.begin(), vec.end());
//   }

//   // 初始 关节角速度 & 角加速度 设为0
//   qDesired_total.front().velocity.setZero();
//   qDesired_total.front().acceleration.setZero();

//   max_time = 0.0;

//   // 启动优化线程
//   casadi_opt_init();
//   std::thread worker(std::bind(&PandaHocbfTestController::thread_task, this));
//   // 线程后台运行
//   worker.detach();

//   return true;
// }

// // 启动动作
// void PandaHocbfTestController::starting(const ros::Time& /* time */) {
//   for (std::size_t i = 0; i < 7; ++i) {
//     initial_pose_[i] = effort_joint_handles_[i].getPosition();
//   }

//   elapsed_time_ = ros::Duration(0.0);
// }

// // 主循环
// void PandaHocbfTestController::update(const ros::Time& /*time*/, const ros::Duration& period) {
//   elapsed_time_ += period;

//   auto start = std::chrono::high_resolution_clock::now();

//   // 采当前关节数据
//   for (std::size_t i = 0; i < 7; i++) {
//     current_state.q[i] = effort_joint_handles_[i].getPosition();
//     current_state.qdot[i] = effort_joint_handles_[i].getVelocity();
//   }

//   if (current_step < qDesired_total.size() - 1) {
//     // 计算当前误差
//     current_state.e_q = qDesired_total[current_step].position - current_state.q;
//     current_state.e_qdot = qDesired_total[current_step].velocity - current_state.qdot;

//     // 更新关节期望速度
//     qDesired_total[current_step + 1].velocity =
//         (qDesired_total[current_step + 1].position - qDesired_total[current_step].position) / dt;
//     // 关节期望速度限幅
//     qDesired_total[current_step + 1].velocity = qDesired_total[current_step + 1]
//                                                     .velocity.array()
//                                                     .max(0.6 * joint_velocity_min_limit.array())
//                                                     .min(0.6 * joint_velocity_max_limit.array());
//     // 更新关节期望加速度
//     qDesired_total[current_step + 1].acceleration =
//         (qDesired_total[current_step + 1].velocity - qDesired_total[current_step].velocity) / dt;
//     // 关节期望加速度限幅
//     qDesired_total[current_step + 1].acceleration = qDesired_total[current_step + 1]
//                                                         .acceleration.array()
//                                                         .max(0.6 * joint_acc_min_limit.array())
//                                                         .min(0.6 * joint_acc_max_limit.array());

//     // cbf避障控制 并且首先执行pd初始化动作
//     // if (cbf_params.CBF_switch) {
//     if (cbf_params.CBF_switch && (current_step > action_init.num_waypoints)) {
//       //   // 优化开启
//       //   casadi_opt_startup = true;
//       // 关节加速度项设为0
//       Eigen::VectorXd qddot = Eigen::VectorXd::Zero(panda_model.nv);
//       // 更新运动学参数
//       pinocchio::forwardKinematics(panda_model, panda_data, current_state.q, current_state.qdot,
//                                    qddot);
//       pinocchio::updateFramePlacements(panda_model, panda_data);

//       // 计算当前末端位置 (matlab中的p1)
//       current_state.X = panda_data.oMf[End_effector_joint_id];
//       current_state.p = current_state.X.translation();
//       // 计算雅可比矩阵 (6x7) 写入 current_state.J
//       current_state.J.resize(6, panda_model.nv);  // nv=7
//       current_state.J.setZero();
//       pinocchio::computeFrameJacobian(panda_model, panda_data, current_state.q,
//                                       End_effector_joint_id, pinocchio::LOCAL, current_state.J);
//       // 计算雅可比矩阵 (matlab中的A1 (3x7)) 写入 current_state.J_p
//       current_state.J_p = current_state.J.topRows(3);

//       // 计算末端速度 pdot1(3x1)
//       current_state.pdot = pinocchio::getFrameVelocity(panda_model, panda_data,
//                                                        End_effector_joint_id, pinocchio::LOCAL)
//                                .linear();
//       // 计算非线性加速度项 B1 (3x1) 写入 current_state.Jdot_qdot 分离非线性项
//       current_state.Jdot_qdot =
//           pinocchio::getFrameAcceleration(panda_model, panda_data, End_effector_joint_id,
//                                           pinocchio::LOCAL)
//               .linear();
//       // 计算安全距离
//       //添加新增参数
//       current_state.h_0 =
//           //   std::pow(2 * obstacle_radius, 2) - (current_state.p -
//           //   obstacle_position).squaredNorm();
//           std::pow(3 * obstacle_radius, 2) - (current_state.p - obstacle_position).squaredNorm();

//       //  名义加速度生成
//       current_state.qddot_norm =
//           qDesired_total[current_step].acceleration +
//           (PD_control.Kp_matrix * current_state.e_q + PD_control.Kd_matrix * current_state.e_qdot);
//       // 名义加速度限幅
//       current_state.qddot_norm = current_state.qddot_norm.array()
//                                      .max(joint_acc_min_limit.array())
//                                      .min(joint_acc_max_limit.array());

//       //添加新增参数
//       // 在update()函数中计算时变参数
//       //   double t_current = elapsed_time_.toSec();
//       double t_current = elapsed_time_.toSec() - dt * action_init.num_waypoints;
//       double xi = std::exp(cbf_params.omega_ * (cbf_params.t_d_ - t_current)) - 1;
//       double xidot =
//           -cbf_params.omega_ * std::exp(cbf_params.omega_ * (cbf_params.t_d_ - t_current));
//       double xiddot = cbf_params.omega_ * cbf_params.omega_ *
//                       std::exp(cbf_params.omega_ * (cbf_params.t_d_ - t_current));
//       cbf_params.C1 = current_state.p - obstacle_position;

//       cbf_params.h1 = current_state.h_0;

//       std::cout << "cbf_params.h1: " << cbf_params.h1 << std::endl;

//       // 避障控制
//       //   末端在球外
//       if (cbf_params.h1 < 0) {
//         // 似乎是对的
//         cbf_params.C1 = -1 * cbf_params.C1;
//         // cbf_params.h1 = -1 * cbf_params.h1;
//         cbf_params.k = cbf_params.h1;
//         // cbf_params.k = 0;
//         cbf_params.k2 = (std::abs(xidot) / xi) * (-2) * cbf_params.C1.dot(current_state.pdot);
//         // cbf_params.k2 = 0;

//         if (is_in_sphere && ~left_sphere) {
//           left_sphere = true;
//         }
//       }
//       // 距离障碍物大于安全距离，无需避障控制
//       //   末端在球内
//       else {
//         cbf_params.k = 0;
//         cbf_params.k2 = 0;

//         if (~is_in_sphere) {
//           is_in_sphere = true;
//         }
//       }

//       //离开球4cm，保持当前位置
//       if (left_sphere && (current_state.h_0 < -0.04)) {
//         if (~stopped_at_outer_sphere) {
//           desire_stopped_joint_waypoint.position = current_state.q;
//           desire_stopped_joint_waypoint.velocity.setZero();
//           desire_stopped_joint_waypoint.acceleration.setZero();
//           stopped_at_outer_sphere = true;
//         }

//         // 计算当前误差
//         current_state.e_q = desire_stopped_joint_waypoint.position - current_state.q;
//         current_state.e_qdot = desire_stopped_joint_waypoint.velocity - current_state.qdot;

//         // 计算当前关节期望力矩（前馈项）
//         desire_stopped_joint_waypoint.torque = pinocchio::rnea(
//             panda_model, panda_data, current_state.q, desire_stopped_joint_waypoint.velocity,
//             desire_stopped_joint_waypoint.acceleration);

//         // 期望力矩限幅
//         // desire_stopped_joint_waypoint.torque = desire_stopped_joint_waypoint.torque.array()
//         //                                            .max(0.5 * joint_torque_min_limit.array())
//         //                                            .min(0.5 * joint_torque_max_limit.array());
//         desire_stopped_joint_waypoint.torque = desire_stopped_joint_waypoint.torque.array()
//                                                    .max(0.001 * joint_torque_min_limit.array())
//                                                    .min(0.001 * joint_torque_max_limit.array());

//         // 计算惯性矩阵（返回上三角）
//         pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
//         // 补全对称矩阵
//         panda_data.M.triangularView<Eigen::StrictlyLower>() =
//             panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();

//         // 关节力矩控制（反馈项）
//         current_state.e_torque = panda_data.M * (PD_control.Kp_matrix * current_state.e_q +
//                                                  PD_control.Kd_matrix * current_state.e_qdot);

//         // 力矩补偿
//         desire_stopped_joint_waypoint.torque += current_state.e_torque;

//         // 期望力矩限幅
//         desire_stopped_joint_waypoint.torque = desire_stopped_joint_waypoint.torque.array()
//                                                    .max(0.8 * joint_torque_min_limit.array())
//                                                    .min(0.8 * joint_torque_max_limit.array());
//         // 当前要输出的力矩
//         qDesired_total[current_step].torque = desire_stopped_joint_waypoint.torque;

//       }

//       //未离开球，正常求解
//       else {
//         // 计算前馈非线性项
//         current_state.nonlinear.setZero();
//         // 计算惯性矩阵（返回上三角）
//         pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
//         // 补全对称矩阵
//         panda_data.M.triangularView<Eigen::StrictlyLower>() =
//             panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();

//         // 修改phi1计算

//         cbf_params.phi1 = -2 * cbf_params.C1.dot(current_state.pdot) +
//                           cbf_params.alpha1 * cbf_params.h1 + (std::abs(xidot) / xi) * cbf_params.k;
//         // 修改D1计算

//         cbf_params.D1 = -2 * current_state.pdot.squaredNorm() +
//                         cbf_params.alpha1 * (-2 * cbf_params.C1.dot(current_state.pdot)) -
//                         (xiddot - xidot * xidot) / (xi * xi) * cbf_params.k + cbf_params.k2 +
//                         cbf_params.alpha2 * cbf_params.phi1;

//         // 计算二次规划目标函数的一次项向量
//         {
//           std::lock_guard<std::mutex> lock(opt_mutex_);
//           cbf_params.f = casadi::DM::vertcat({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//         }

//         // 求约束相关参数

//         cbf_params.A_vec = (2 * cbf_params.C1.transpose() * current_state.J_p).transpose();
//         {
//           std::lock_guard<std::mutex> lock(opt_mutex_);
//           cbf_params.A = casadi::DM::horzcat(
//               {cbf_params.A_vec[0], cbf_params.A_vec[1], cbf_params.A_vec[2], cbf_params.A_vec[3],
//                cbf_params.A_vec[4], cbf_params.A_vec[5], cbf_params.A_vec[6]});
//         }

//         cbf_params.b_value =
//             -2 * cbf_params.C1.transpose() * current_state.Jdot_qdot + cbf_params.D1;
//         // cbf_params.b_value = cbf_params.D1;
//         {
//           std::lock_guard<std::mutex> lock(opt_mutex_);
//           cbf_params.b = cbf_params.b_value;
//         }

//         //   std::cout << "cbf_params.D1: " << cbf_params.D1 << std::endl;

//         // std::cout << "cbf_params.b: " << cbf_params.b << std::endl;

//         // 优化开启
//         casadi_opt_startup = true;

//         // 检测优化求解是否完成
//         // 原子操作，无需锁
//         int current_opt = opt_step_current.load(std::memory_order_acquire);
//         if (current_opt != opt_step_last.load(std::memory_order_relaxed)) {
//           opt_step_last.store(current_opt, std::memory_order_relaxed);
//           // 赋值期望力矩
//           {
//             std::lock_guard<std::mutex> lock(opt_mutex_);
//             // qDesired_total[current_step].torque =
//             //     Eigen::Map<PandaHocbfTestController::Vector7d>(cbf_params.output_vec.data());

//             filter_new_mearsurement =
//                 Eigen::Map<PandaHocbfTestController::Vector7d>(cbf_params.output_vec.data());
//           }
//         }
//         // } else {
//         //   qDesired_total[current_step].torque = qDesired_total[current_step - 1].torque;
//         // }
//         // 滤波输出
//         //   qddotSafe = filter_new_mearsurement
//         if (filter_new_mearsurement.norm() > 0.001) {
//           // 计算前馈
//           PandaHocbfTestController::Vector7d InverseDyn =
//               pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
//                               qDesired_total[current_step].acceleration);
//           // 不使用非线性项
//           InverseDyn.setZero();

//           // 新优化结果转为力矩，关节加速度转为关节力矩
//           // PandaHocbfTestController::Vector7d new_tau =
//           //     panda_data.M * filter_new_mearsurement + InverseDyn;

//           PandaHocbfTestController::Vector7d new_tau = panda_data.M * filter_new_mearsurement;

//           // 滤波得到最终关节力
//           qDesired_total[current_step].torque =
//               filter_alpha * qDesired_total[current_step - 1].torque + (1 - filter_alpha) * new_tau;
//         } else {
//           qDesired_total[current_step].torque = qDesired_total[current_step - 1].torque;
//         }

//         // 已经进入球内，并且还未停止，加入扰动
//         // if (is_in_sphere && ~stopped_at_outer_sphere) {
//         // if (current_state.h_0 > -0.005) {
//         //   left_sphere = true;
//         //   // 这里添加扰动
//         //   PandaHocbfTestController::Vector7d d_v0;
//         //   d_v0.setZero();
//         //   PandaHocbfTestController::Vector7d d_a0;
//         //   d_a0.setZero();
//         //   PandaHocbfTestController::Vector7d tau_disturbance;

//         //   PandaHocbfTestController::Vector7d Gvec;
//         //   PandaHocbfTestController::Vector7d Cvec;
//         //   // 在速度、加速度均为零的情况下，计算出的就是重力项
//         //   Gvec = pinocchio::rnea(panda_model, panda_data, current_state.q, d_v0, d_a0);
//         //   // 在加速度为零且无外力（重力也设置为零）的情况下，计算出的就是科里奥利和离心力项
//         //   Cvec =
//         //       pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
//         //       d_a0);
//         //   // 计算扰动
//         //   tau_disturbance = -0.015 * Gvec - 0.015 * Cvec;
//         //   // 加入扰动
//         //   qDesired_total[current_step].torque += tau_disturbance;
//         // }
//       }
//     }
//     // 一般pd控制
//     else {
//       // 计算当前关节期望力矩（前馈项）
//       qDesired_total[current_step].torque =
//           pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
//                           qDesired_total[current_step].acceleration);

//       // 期望力矩限幅
//       //   qDesired_total[current_step].torque = qDesired_total[current_step]
//       //                                             .torque.array()
//       //                                             .max(0.5 * joint_torque_min_limit.array())
//       //                                             .min(0.5 * joint_torque_max_limit.array());

//       qDesired_total[current_step].torque = qDesired_total[current_step]
//                                                 .torque.array()
//                                                 .max(0.001 * joint_torque_min_limit.array())
//                                                 .min(0.001 * joint_torque_max_limit.array());

//       // 计算惯性矩阵（返回上三角）
//       pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
//       // 补全对称矩阵
//       panda_data.M.triangularView<Eigen::StrictlyLower>() =
//           panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();
//       // 关节力矩控制（反馈项）
//       current_state.e_torque = panda_data.M * (PD_control.Kp_matrix * current_state.e_q +
//                                                PD_control.Kd_matrix * current_state.e_qdot);

//       // 力矩补偿
//       qDesired_total[current_step].torque += current_state.e_torque;
//     }

//     // 期望力矩限幅
//     qDesired_total[current_step].torque = qDesired_total[current_step]
//                                               .torque.array()
//                                               .max(0.8 * joint_torque_min_limit.array())
//                                               .min(0.8 * joint_torque_max_limit.array());

//     // 发送力矩指令到机械臂
//     for (int i = 0; i < 7; i++) {
//       effort_joint_handles_[i].setCommand(qDesired_total[current_step].torque[i]);
//     }

//     // std::cout << "max_time: " << max_time << std::endl;

//   }

//   else {
//     casadi_opt_startup = false;
//     // 设置结束位置等于最后执行的位置
//     if (current_step == (qDesired_total.size() - 1)) {
//       qDesired_total.back().position = current_state.q;
//     }

//     // 结束期望速度和加速度为0
//     qDesired_total.back().velocity.setZero();
//     qDesired_total.back().acceleration.setZero();

//     // 计算当前误差
//     current_state.e_q = qDesired_total.back().position - current_state.q;
//     current_state.e_qdot = qDesired_total.back().velocity - current_state.qdot;

//     // 计算当前关节期望力矩（前馈项）
//     qDesired_total.back().torque =
//         pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
//                         qDesired_total.back().acceleration);

//     // 期望力矩限幅
//     // qDesired_total.back().torque = qDesired_total.back()
//     //                                    .torque.array()
//     //                                    .max(0.5 * joint_torque_min_limit.array())
//     //                                    .min(0.5 * joint_torque_max_limit.array());

//     qDesired_total.back().torque = qDesired_total.back()
//                                        .torque.array()
//                                        .max(0.001 * joint_torque_min_limit.array())
//                                        .min(0.001 * joint_torque_max_limit.array());

//     // 计算惯性矩阵（返回上三角）
//     pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
//     // 补全对称矩阵
//     panda_data.M.triangularView<Eigen::StrictlyLower>() =
//         panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();
//     // 关节力矩控制（反馈项）
//     current_state.e_torque = panda_data.M * (PD_control.Kp_matrix * current_state.e_q +
//                                              PD_control.Kd_matrix * current_state.e_qdot);

//     // 力矩补偿
//     qDesired_total.back().torque += current_state.e_torque;

//     // 期望力矩限幅
//     qDesired_total.back().torque = qDesired_total.back()
//                                        .torque.array()
//                                        .max(0.8 * joint_torque_min_limit.array())
//                                        .min(0.8 * joint_torque_max_limit.array());

//     // 发送力矩指令到机械臂
//     for (int i = 0; i < 7; i++) {
//       effort_joint_handles_[i].setCommand(qDesired_total.back().torque[i]);
//     }
//   }

//   current_step++;

//   // std::cout << "current_step: " << current_step << std::endl;
//   //   std::cout << "current_state.q: " << current_state.q << std::endl;

//   // 计算时间
//   auto end = std::chrono::high_resolution_clock::now();
//   double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

//   max_time = std::max(max_time, duration);
// }

// // 关节轨迹插补定义
// std::vector<PandaHocbfTestController::Joint_Waypoint> PandaHocbfTestController::Joint_Trajectory(
//     PandaHocbfTestController::Vector7d& qstart,
//     PandaHocbfTestController::Vector7d& qend,
//     double Tf,
//     int N,
//     int method) {
//   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
//   traj.resize(N);
//   for (int i = 0; i < N; i++) {
//     double s = PandaHocbfTestController::QuinticTimeScaling(Tf, i * dt);
//     traj[i].position = qstart + s * (qend - qstart);
//   }
//   return traj;
// }

// // 笛卡尔轨迹插补定义
// std::vector<PandaHocbfTestController::Actuator_Waypoint>
// PandaHocbfTestController::Cartesian_Trajectory(PandaHocbfTestController::Actuator_Action& action) {
//   std::vector<PandaHocbfTestController::Actuator_Waypoint> traj{};
//   traj.resize(action.num_waypoints);

//   for (int i = 0; i < action.num_waypoints; i++) {
//     double s = PandaHocbfTestController::QuinticTimeScaling(action.Tf_duration, i * dt);
//     traj[i].R = action.Rstart *
//                 pinocchio::exp3((pinocchio::log3(action.Rstart.transpose() * action.Rend) * s));
//     traj[i].p = action.pstart + s * (action.pend - action.pstart);
//     traj[i].X = pinocchio::SE3(traj[i].R, traj[i].p);
//   }

//   return traj;
// }

// // 逆运动学迭代求解
// std::vector<PandaHocbfTestController::Joint_Waypoint> PandaHocbfTestController::Inverse_Kinematics(
//     PandaHocbfTestController::Actuator_Action& action,
//     PandaHocbfTestController::Vector7d& q_init) {
//   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
//   traj.resize(action.num_waypoints);
//   traj[0].position = q_init;
//   // 当前位姿的Jacobian矩阵panda_model.nv=7
//   pinocchio::Data::Matrix6x J(6, panda_model.nv);
//   // se3 error
//   PandaHocbfTestController::Vector6d err;
//   // 目标关节速度
//   Eigen::VectorXd v(panda_model.nv);
//   // 修正jacobian
//   pinocchio::Data::Matrix6 Jlog;
//   // 正则化的jacobian
//   pinocchio::Data::Matrix6 JJt;

//   for (int i = 0; i < action.num_waypoints - 1; i++) {
//     // 初始为0
//     J.setZero();
//     Jlog.setZero();
//     JJt.setZero();
//     // 目标位姿
//     pinocchio::SE3 oMdes = action.X_desired[i + 1].X;
//     // 坐标变换到当前参考系
//     pinocchio::SE3 iMd = action.X_desired[i].X.actInv(oMdes);
//     // 计算se3 error
//     err = pinocchio::log6(iMd).toVector();
//     // 用当前的q计算jacobian，End_effector_joint_id wrt base
//     pinocchio::computeFrameJacobian(panda_model, panda_data, traj[i].position,
//                                     End_effector_joint_id, pinocchio::LOCAL, J);

//     // 正则化jacobian，解决奇异值问题
//     pinocchio::Jlog6(iMd.inverse(), Jlog);
//     J = -Jlog * J;
//     JJt.noalias() = J * J.transpose();
//     JJt.diagonal().array() += J_damping;
//     v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
//     traj[i + 1].position = pinocchio::integrate(panda_model, traj[i].position, v);
//   }
//   return traj;
// }

// // 五次样条插值,返回平滑系数
// double PandaHocbfTestController::QuinticTimeScaling(double total_time, double current_time) {
//   if (total_time <= 0.0)
//     return 0.0;
//   double t = std::max(0.0, std::min(current_time / total_time, 1.0));  // 限制在[0,1]区间
//   double s = 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
//   return s;
// }

// // 静止姿态动作
// std::vector<PandaHocbfTestController::Joint_Waypoint> PandaHocbfTestController::Joint_Stay(
//     PandaHocbfTestController::Vector7d& qstay,
//     int N,
//     double Tf_duration) {
//   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
//   traj.resize(N);
//   for (int i = 0; i < N; i++) {
//     traj[i].position = qstay;
//   }
//   return traj;
// }

// // 初始化优化问题
// void PandaHocbfTestController::casadi_opt_init() {
//   casadi_cbf.opti = casadi::Opti();
//   casadi_cbf.tau = casadi_cbf.opti.variable(7, 1);
//   casadi_cbf.H = casadi_cbf.opti.parameter(7, 7);  // 参数占位符
//   casadi_cbf.f = casadi_cbf.opti.parameter(7, 1);
//   casadi_cbf.A = casadi_cbf.opti.parameter(1, 7);  // 参数占位符
//   casadi_cbf.b = casadi_cbf.opti.parameter(1, 1);
//   casadi_cbf.opti.minimize(
//       0.5 *
//           casadi::MX::mtimes(casadi::MX::mtimes(casadi_cbf.tau.T(), casadi_cbf.H), casadi_cbf.tau) +
//       casadi::MX::mtimes(casadi_cbf.f.T(), casadi_cbf.tau));  // 添加目标函数

//   // 约束1：线性约束
//   casadi_cbf.opti.subject_to(casadi::MX::mtimes(casadi_cbf.A, casadi_cbf.tau) <= casadi_cbf.b);

//   // 约束2：上下界
//   casadi_cbf.opti.subject_to(cbf_params.lbU <= casadi_cbf.tau <= cbf_params.ubU);
//   casadi_cbf.opti.solver("ipopt", casadiOptions, solverOptions);
// }

// // 优化问题求解
// void PandaHocbfTestController::thread_task() {
//   while (true) {
//     if (casadi_opt_startup) {
//       // std::cout << "opti_solving " << std::endl;

//       {
//         std::lock_guard<std::mutex> lock(opt_mutex_);
//         casadi_cbf.opti.set_value(casadi_cbf.H, cbf_params.H);
//         casadi_cbf.opti.set_value(casadi_cbf.f, cbf_params.f);
//         casadi_cbf.opti.set_value(casadi_cbf.A, cbf_params.A);
//         casadi_cbf.opti.set_value(casadi_cbf.b, cbf_params.b);
//       }

//       auto solution = casadi_cbf.opti.solve();
//       casadi::DM output = solution.value(casadi_cbf.tau);

//       {
//         std::lock_guard<std::mutex> lock(opt_mutex_);
//         cbf_params.output_vec = std::vector<double>(output);
//       }

//       // 优化成功，更新step
//       opt_step_current.store(opt_step_last.load() + 1, std::memory_order_release);
//     }

//     else {
//       casadi_cbf.opti.set_value(casadi_cbf.H, 2 * casadi::DM::eye(7));
//       casadi_cbf.opti.set_value(casadi_cbf.f, casadi::DM::zeros(7, 1));
//       casadi_cbf.opti.set_value(casadi_cbf.A, casadi::DM::zeros(1, 7));
//       casadi_cbf.opti.set_value(casadi_cbf.b, casadi::DM::zeros(1, 1));
//       auto solution = casadi_cbf.opti.solve();
//     }
//   }
// }

// }  // namespace franka_example_controllers

// PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PandaHocbfTestController,
//                        controller_interface::ControllerBase)

// // // Copyright (c) 2017 Franka Emika GmbH
// // // Use of this source code is governed by the Apache-2.0 license, see LICENSE
// // #include <franka_example_controllers/panda_hocbf_test_controller.h>

// // #include <cmath>

// // #include <controller_interface/controller_base.h>
// // #include <hardware_interface/hardware_interface.h>
// // #include <hardware_interface/joint_command_interface.h>
// // #include <pluginlib/class_list_macros.h>
// // // #include <ros/ros.h>
// // #include <rosconsole/macros_generated.h>
// // #include <algorithm>
// // #include <cmath>

// // namespace franka_example_controllers {
// // bool PandaHocbfTestController::init(hardware_interface::RobotHW* robot_hardware,
// //                                     ros::NodeHandle& node_handle) {
// //   // 获取关节力接口
// //   effort_joint_interface_ = robot_hardware->get<hardware_interface::EffortJointInterface>();

// //   if (effort_joint_interface_ == nullptr) {
// //     ROS_ERROR("Error getting position or effort joint interface from hardware!");
// //     return false;
// //   }

// //   // 获取关节名称
// //   std::vector<std::string> joint_names;
// //   if (!node_handle.getParam("joint_names", joint_names)) {
// //     ROS_ERROR("Could not parse joint names");
// //   }
// //   if (joint_names.size() != 7) {
// //     ROS_ERROR_STREAM("Wrong number of joint names, got " << joint_names.size()
// //                                                          << " instead of 7 names!");
// //     return false;
// //   }

// //   // 初始化关节句柄
// //   effort_joint_handles_.resize(7);

// //   for (std::size_t i = 0; i < 7; ++i) {
// //     try {
// //       // 获取关节句柄
// //       effort_joint_handles_[i] = effort_joint_interface_->getHandle(joint_names[i]);
// //     } catch (const hardware_interface::HardwareInterfaceException& e) {
// //       ROS_ERROR_STREAM("Exception getting joint handles: " << e.what());
// //       return false;
// //     }
// //   }

// //   // 检查模型初始位置
// //   //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
// //   //   for (std::size_t i = 0; i < q_start.size(); i++) {
// //   //     if (std::abs(effort_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
// //   //       ROS_ERROR_STREAM(
// //   //           "JointPositionExampleController: Robot is not in the expected starting position
// //   for "
// //   //           "running this example. Run `roslaunch franka_example_controllers
// //   move_to_start.launch
// //   //           " "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
// //   //       return false;
// //   //     }
// //   //   }
// //   std::array<double, 7> q_start;
// //   //   std::array<double, 7> q_start_desired{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
// //   for (std::size_t i = 0; i < q_start.size(); i++) {
// //     q_start[i] = effort_joint_handles_[i].getPosition();
// //   }

// //   // gazebo控制周期
// //   dt = 0.001;

// //   // 初始化步数
// //   current_step = 0;

// //   // 初始化逆运动学阻尼
// //   J_damping = 1e-6;

// //   // 滤波器参数
// //   filter_alpha = 0.2;

// //   // 开启避障
// //   //   cbf_params.CBF_switch = true;
// //   cbf_params.CBF_switch = false;

// //   // 初始化cbf参数
// //   cbf_params.H = 2 * casadi::DM::eye(7);
// //   cbf_params.ubU = 0.8 * casadi::DM::vertcat({
// //                              87.0, 87.0, 87.0, 87.0,  // 前4个关节扭矩上限
// //                              12.0, 12.0, 12.0         // 后3个关节扭矩上限
// //                          });
// //   cbf_params.lbU = (-1) * cbf_params.ubU;
// //   cbf_params.alpha1 = 5;
// //   cbf_params.alpha2 = 30;
// //   casadi_opt_startup = false;
// //   opt_step_last = 0;
// //   opt_step_current = 0;

// //   // 初始化优化参数

// //   casadi::Dict jitOptions;
// //   jitOptions["flags"] = "-O3";
// //   jitOptions["verbose"] = true;

// //   casadiOptions["expand"] = true;
// //   casadiOptions["error_on_fail"] = true;
// //   casadiOptions["print_time"] = false;

// //   casadiOptions["jit"] = true;
// //   casadiOptions["compiler"] = "shell";
// //   casadiOptions["jit_options"] = jitOptions;

// //   solverOptions["print_level"] = 0;
// //   solverOptions["max_iter"] = 3000;
// //   solverOptions["tol"] = 1e-8;
// //   solverOptions["linear_solver"] = "ma57";

// //   solverOptions["warm_start_init_point"] = "yes";
// //   solverOptions["warm_start_bound_push"] = 1e-8;
// //   solverOptions["warm_start_mult_bound_push"] = 1e-8;
// //   solverOptions["mu_init"] = 1e-5;
// //   solverOptions["bound_relax_factor"] = 1e-9;

// //   // 障碍物位置
// //   obstacle_position = Eigen::Vector3d(0.53, 0.0, 0.2);

// //   // 初始化夹爪半径
// //   obstacle_radius = 0.03;

// //   // 初始化控制参数
// //   //   PD_control.Kp_matrix.diagonal() = 10 * PandaHocbfTestController::Vector7d::Ones() / dt;
// //   PD_control.Kp_matrix.diagonal() = 300.0 * PandaHocbfTestController::Vector7d::Ones();
// //   //   PD_control.Kd_matrix.diagonal() = 0.4 * PandaHocbfTestController::Vector7d::Ones() / dt;
// //   PD_control.Kd_matrix.diagonal() = 30.0 * PandaHocbfTestController::Vector7d::Ones();

// //   // 初始化关节限幅
// //   joint_velocity_max_limit =
// //       (PandaHocbfTestController::Vector7d() << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)
// //           .finished();
// //   joint_velocity_min_limit = (-1) * joint_velocity_max_limit;

// //   joint_acc_max_limit =
// //       (PandaHocbfTestController::Vector7d() << 15, 7.5, 10, 12.5, 15, 20, 20).finished();
// //   joint_acc_min_limit = (-1) * joint_acc_max_limit;

// //   joint_torque_max_limit =
// //       (PandaHocbfTestController::Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();
// //   joint_torque_min_limit = (-1) * joint_torque_max_limit;

// //   // 绝对路径
// //   urdf_filename = std::string(
// //       "/home/robot/franka_ros_ws/src/franka_ros/franka_description/robots/panda/panda.urdf");
// //   pinocchio::urdf::buildModel(urdf_filename, panda_model);
// //   panda_data = pinocchio::Data(panda_model);

// //   // 初始化动作
// //   action_init.qstart = Eigen::Map<const PandaHocbfTestController::Vector7d>(q_start.data());
// //   action_init.qend =
// //       (PandaHocbfTestController::Vector7d() << 0, 0, 0, -1 * M_PI_2, 0, M_PI_2,
// //       M_PI_4).finished();

// //   action_init.Tf_duration = 2.0;
// //   action_init.num_waypoints = (int)(action_init.Tf_duration / dt + 1);
// //   action_init.method = 5;
// //   action_init.qDesired =
// //       Joint_Trajectory(action_init.qstart, action_init.qend, action_init.Tf_duration,
// //                        action_init.num_waypoints, action_init.method);

// //   // 初始化静止姿态动作
// //   stay_init.qstay = action_init.qend;
// //   stay_init.Tf_duration = 0.4;
// //   stay_init.num_waypoints = (int)(stay_init.Tf_duration / dt + 1);
// //   stay_init.qDesired = Joint_Stay(stay_init.qstay, stay_init.num_waypoints,
// //   stay_init.Tf_duration);

// //   // 动作0
// //   action0.qstart = stay_init.qstay;
// //   action0.qend = (PandaHocbfTestController::Vector7d() << 0, -40 * M_PI / 180, 0, -130 * M_PI /
// //   180,
// //                   0, M_PI_2, M_PI_4)
// //                      .finished();

// //   action0.Tf_duration = 3.0;
// //   action0.num_waypoints = (int)(action0.Tf_duration / dt + 1);
// //   action0.method = 5;
// //   action0.qDesired = Joint_Trajectory(action0.qstart, action0.qend, action0.Tf_duration,
// //                                       action0.num_waypoints, action0.method);

// //   // 动作0静止
// //   stay0.qstay = action0.qend;
// //   stay0.Tf_duration = 0.4;
// //   stay0.num_waypoints = (int)(stay0.Tf_duration / dt + 1);
// //   stay0.qDesired = Joint_Stay(stay0.qstay, stay0.num_waypoints, stay0.Tf_duration);

// //   // 动作1
// //   // 运动学解算
// //   End_effector_joint_id = panda_model.getFrameId("panda_hand_tcp_joint");
// //   pinocchio::framesForwardKinematics(panda_model, panda_data, stay0.qstay);

// //   action1.Xstart = panda_data.oMf[End_effector_joint_id];
// //   action1.Rstart = action1.Xstart.rotation();
// //   action1.pstart = action1.Xstart.translation();

// //   action1.Rend = action1.Rstart * pinocchio::exp3(Eigen::Vector3d(0, 0, (0 * M_PI / 180)));
// //   action1.pend = action1.pstart + Eigen::Vector3d(0.0, 0.0, -0.3);
// //   action1.Xend = pinocchio::SE3(action1.Rend, action1.pend);

// //   action1.Tf_duration = 3.0;
// //   action1.num_waypoints = (int)(action1.Tf_duration / dt + 1);
// //   action1.method = 5;
// //   action1.X_desired = Cartesian_Trajectory(action1);

// //   //   逆运动学迭代
// //   action1.qDesired = Inverse_Kinematics(action1, stay0.qstay);

// //   // 动作1静止
// //   stay1.qstay = action1.qDesired.back().position;
// //   stay1.Tf_duration = 0.4;
// //   stay1.num_waypoints = (int)(stay1.Tf_duration / dt + 1);
// //   stay1.qDesired = Joint_Stay(stay1.qstay, stay1.num_waypoints, stay1.Tf_duration);

// //   // 动作2
// //   //   pinocchio::framesForwardKinematics(panda_model, panda_data, stay1.qstay);
// //   action2.Xstart = action1.Xend;
// //   action2.Rstart = action2.Xstart.rotation();
// //   action2.pstart = action2.Xstart.translation();

// //   action2.Rend = action2.Rstart * pinocchio::exp3(Eigen::Vector3d(0, 0, (0 * M_PI / 180)));
// //   action2.pend = action2.pstart + Eigen::Vector3d(0.45, 0.0, 0.0);
// //   action2.Xend = pinocchio::SE3(action2.Rend, action2.pend);

// //   action2.Tf_duration = 5;
// //   action2.num_waypoints = (int)(action2.Tf_duration / dt + 1);
// //   action2.method = 5;
// //   action2.X_desired = Cartesian_Trajectory(action2);

// //   //   逆运动学迭代
// //   action2.qDesired = Inverse_Kinematics(action2, stay1.qstay);

// //   // 动作2静止
// //   stay2.qstay = action2.qDesired.back().position;
// //   stay2.Tf_duration = 0.4;
// //   stay2.num_waypoints = (int)(stay2.Tf_duration / dt + 1);
// //   stay2.qDesired = Joint_Stay(stay2.qstay, stay2.num_waypoints, stay2.Tf_duration);

// //   // 动作拼接
// //   for (const auto& vec :
// //        {action_init.qDesired, stay_init.qDesired, action0.qDesired, stay0.qDesired,
// //         action1.qDesired, stay1.qDesired, action2.qDesired, stay2.qDesired}) {
// //     qDesired_total.insert(qDesired_total.end(), vec.begin(), vec.end());
// //   }

// //   // 初始 关节角速度 & 角加速度 设为0
// //   qDesired_total.front().velocity.setZero();
// //   qDesired_total.front().acceleration.setZero();

// //   max_time = 0.0;

// //   // 启动优化线程
// //   casadi_opt_init();
// //   std::thread worker(std::bind(&PandaHocbfTestController::thread_task, this));
// //   // 线程后台运行
// //   worker.detach();

// //   return true;
// // }

// // // 启动动作
// // void PandaHocbfTestController::starting(const ros::Time& /* time */) {
// //   for (std::size_t i = 0; i < 7; ++i) {
// //     initial_pose_[i] = effort_joint_handles_[i].getPosition();
// //   }

// //   elapsed_time_ = ros::Duration(0.0);
// // }

// // // 主循环
// // void PandaHocbfTestController::update(const ros::Time& /*time*/, const ros::Duration& period) {
// //   elapsed_time_ += period;

// //   auto start = std::chrono::high_resolution_clock::now();

// //   // 采当前关节数据
// //   for (std::size_t i = 0; i < 7; i++) {
// //     current_state.q[i] = effort_joint_handles_[i].getPosition();
// //     current_state.qdot[i] = effort_joint_handles_[i].getVelocity();
// //   }

// //   if (current_step < qDesired_total.size() - 1) {
// //     // 计算当前误差
// //     current_state.e_q = qDesired_total[current_step].position - current_state.q;
// //     current_state.e_qdot = qDesired_total[current_step].velocity - current_state.qdot;

// //     // 更新关节期望速度
// //     qDesired_total[current_step + 1].velocity =
// //         (qDesired_total[current_step + 1].position - qDesired_total[current_step].position) / dt;
// //     // 关节期望速度限幅
// //     qDesired_total[current_step + 1].velocity = qDesired_total[current_step + 1]
// //                                                     .velocity.array()
// //                                                     .max(0.6 * joint_velocity_min_limit.array())
// //                                                     .min(0.6 * joint_velocity_max_limit.array());
// //     // 更新关节期望加速度
// //     qDesired_total[current_step + 1].acceleration =
// //         (qDesired_total[current_step + 1].velocity - qDesired_total[current_step].velocity) / dt;
// //     // 关节期望加速度限幅
// //     qDesired_total[current_step + 1].acceleration = qDesired_total[current_step + 1]
// //                                                         .acceleration.array()
// //                                                         .max(0.6 * joint_acc_min_limit.array())
// //                                                         .min(0.6 * joint_acc_max_limit.array());

// //     // cbf避障控制
// //     if (cbf_params.CBF_switch) {
// //       // 关节加速度项设为0
// //       Eigen::VectorXd qddot = Eigen::VectorXd::Zero(panda_model.nv);
// //       // 更新运动学参数
// //       pinocchio::forwardKinematics(panda_model, panda_data, current_state.q, current_state.qdot,
// //                                    qddot);
// //       pinocchio::updateFramePlacements(panda_model, panda_data);

// //       // 计算当前末端位置 (matlab中的p1)
// //       current_state.X = panda_data.oMf[End_effector_joint_id];
// //       current_state.p = current_state.X.translation();
// //       // 计算雅可比矩阵 (6x7) 写入 current_state.J
// //       current_state.J.resize(6, panda_model.nv);  // nv=7
// //       current_state.J.setZero();
// //       pinocchio::computeFrameJacobian(panda_model, panda_data, current_state.q,
// //                                       End_effector_joint_id, pinocchio::LOCAL, current_state.J);
// //       // 计算雅可比矩阵 (matlab中的A1 (3x7)) 写入 current_state.J_p
// //       current_state.J_p = current_state.J.topRows(3);

// //       // 计算末端速度 pdot1(3x1)
// //       current_state.pdot = pinocchio::getFrameVelocity(panda_model, panda_data,
// //                                                        End_effector_joint_id, pinocchio::LOCAL)
// //                                .linear();
// //       // 计算非线性加速度项 B1 (3x1) 写入 current_state.Jdot_qdot 分离非线性项
// //       current_state.Jdot_qdot =
// //           pinocchio::getFrameAcceleration(panda_model, panda_data, End_effector_joint_id,
// //                                           pinocchio::LOCAL)
// //               .linear();
// //       // 计算安全距离
// //       current_state.h_0 =
// //           (current_state.p - obstacle_position).squaredNorm() - std::pow(5 * obstacle_radius, 2);

// //       //  名义加速度生成
// //       current_state.qddot_norm =
// //           qDesired_total[current_step].acceleration +
// //           (PD_control.Kp_matrix * current_state.e_q + PD_control.Kd_matrix *
// //           current_state.e_qdot);
// //       // 名义加速度限幅
// //       current_state.qddot_norm = current_state.qddot_norm.array()
// //                                      .max(joint_acc_min_limit.array())
// //                                      .min(joint_acc_max_limit.array());

// //       // 避障控制
// //       if (current_state.h_0 <= 0) {
// //         // 计算前馈非线性项
// //         current_state.nonlinear.setZero();
// //         // current_state.nonlinear =
// //         //     pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
// //         //                     qDesired_total[current_step].acceleration);
// //         // 计算惯性矩阵（返回上三角）
// //         pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
// //         // 补全对称矩阵
// //         panda_data.M.triangularView<Eigen::StrictlyLower>() =
// //             panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();
// //         // 计算二次规划目标函数的一次项向量
// //         cbf_params.f_vec = -2 * (panda_data.M * current_state.qddot_norm +
// //         current_state.nonlinear);

// //         {
// //           std::lock_guard<std::mutex> lock(opt_mutex_);
// //           cbf_params.f = casadi::DM::vertcat(
// //               {cbf_params.f_vec[0], cbf_params.f_vec[1], cbf_params.f_vec[2],
// //               cbf_params.f_vec[3],
// //                cbf_params.f_vec[4], cbf_params.f_vec[5], cbf_params.f_vec[6]});
// //         }
// //         // 求约束相关参数
// //         cbf_params.h1 = (current_state.p - obstacle_position).squaredNorm() -
// //                         std::pow(3.5 * obstacle_radius, 2);

// //         cbf_params.C1 = current_state.p - obstacle_position;

// //         cbf_params.phi1 =
// //             2 * cbf_params.C1.transpose() * current_state.pdot + cbf_params.alpha1 *
// //             cbf_params.h1;

// //         cbf_params.D1 = current_state.pdot.squaredNorm() +
// //                         0.5 * (cbf_params.alpha1 + cbf_params.alpha2) * cbf_params.phi1 -
// //                         0.5 * cbf_params.alpha1 * cbf_params.alpha1 * cbf_params.h1;

// //         cbf_params.A_vec =
// //             ((-1) * cbf_params.C1.transpose() * current_state.J_p * panda_data.M.inverse())
// //                 .transpose();
// //         {
// //           std::lock_guard<std::mutex> lock(opt_mutex_);
// //           cbf_params.A = casadi::DM::horzcat(
// //               {cbf_params.A_vec[0], cbf_params.A_vec[1], cbf_params.A_vec[2],
// //               cbf_params.A_vec[3],
// //                cbf_params.A_vec[4], cbf_params.A_vec[5], cbf_params.A_vec[6]});
// //         }

// //         cbf_params.b_value = cbf_params.C1.transpose() * current_state.Jdot_qdot + cbf_params.D1
// //         -
// //                              cbf_params.C1.transpose() * current_state.J_p *
// //                                  panda_data.M.inverse() * current_state.nonlinear;

// //         {
// //           std::lock_guard<std::mutex> lock(opt_mutex_);
// //           cbf_params.b = cbf_params.b_value;
// //         }

// //         // 优化开启
// //         casadi_opt_startup = true;

// //         // 检测优化求解是否完成
// //         // 原子操作，无需锁
// //         int current_opt = opt_step_current.load(std::memory_order_acquire);
// //         if (current_opt != opt_step_last.load(std::memory_order_relaxed)) {
// //           opt_step_last.store(current_opt, std::memory_order_relaxed);
// //           // 赋值期望力矩
// //           {
// //             std::lock_guard<std::mutex> lock(opt_mutex_);
// //             // qDesired_total[current_step].torque =
// //             //     Eigen::Map<PandaHocbfTestController::Vector7d>(cbf_params.output_vec.data());

// //             filter_new_mearsurement =
// //                 Eigen::Map<PandaHocbfTestController::Vector7d>(cbf_params.output_vec.data());
// //           }
// //         }
// //         // } else {
// //         //   qDesired_total[current_step].torque = qDesired_total[current_step - 1].torque;
// //         // }
// //         // 滤波输出
// //         if (filter_new_mearsurement.norm() > 0.001) {
// //           qDesired_total[current_step].torque =
// //               filter_alpha * qDesired_total[current_step - 1].torque +
// //               (1 - filter_alpha) * filter_new_mearsurement;
// //         } else {
// //           qDesired_total[current_step].torque = qDesired_total[current_step - 1].torque;
// //         }

// //       }
// //       // 距离障碍物大于安全距离，无需避障控制
// //       else {
// //         // 优化关闭
// //         casadi_opt_startup = false;
// //         // 计算前馈非线性项
// //         current_state.nonlinear.setZero();
// //         // current_state.nonlinear =
// //         //     pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
// //         //     qddot);
// //         // 计算惯性矩阵（返回上三角）
// //         pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
// //         // 补全对称矩阵
// //         panda_data.M.triangularView<Eigen::StrictlyLower>() =
// //             panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();

// //         qDesired_total[current_step].torque =
// //             panda_data.M * current_state.qddot_norm + current_state.nonlinear;
// //       }

// //     }
// //     // 一般pd控制
// //     else {
// //       // 计算当前关节期望力矩（前馈项）
// //       qDesired_total[current_step].torque =
// //           pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
// //                           qDesired_total[current_step].acceleration);

// //       // 期望力矩限幅
// //     //   qDesired_total[current_step].torque = qDesired_total[current_step]
// //     //                                             .torque.array()
// //     //                                             .max(0.5 * joint_torque_min_limit.array())
// //     //                                             .min(0.5 * joint_torque_max_limit.array());
// //       qDesired_total[current_step].torque = qDesired_total[current_step]
// //                                                 .torque.array()
// //                                                 .max(0.01 * joint_torque_min_limit.array())
// //                                                 .min(0.01 * joint_torque_max_limit.array());

// //       // 计算惯性矩阵（返回上三角）
// //       pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
// //       // 补全对称矩阵
// //       panda_data.M.triangularView<Eigen::StrictlyLower>() =
// //           panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();
// //       // 关节力矩控制（反馈项）
// //       current_state.e_torque = panda_data.M * (PD_control.Kp_matrix * current_state.e_q +
// //                                                PD_control.Kd_matrix * current_state.e_qdot);

// //       // 力矩补偿
// //       qDesired_total[current_step].torque += current_state.e_torque;
// //     }

// //     // 期望力矩限幅
// //     qDesired_total[current_step].torque = qDesired_total[current_step]
// //                                               .torque.array()
// //                                               .max(0.8 * joint_torque_min_limit.array())
// //                                               .min(0.8 * joint_torque_max_limit.array());

// //     // 发送力矩指令到机械臂
// //     for (int i = 0; i < 7; i++) {
// //       effort_joint_handles_[i].setCommand(qDesired_total[current_step].torque[i]);
// //     }

// //     std::cout << "max_time: " << max_time << std::endl;

// //   }

// //   else {
// //     // 设置结束位置等于最后执行的位置
// //     if (current_step == (qDesired_total.size() - 1)) {
// //       qDesired_total.back().position = current_state.q;
// //     }

// //     // 结束期望速度和加速度为0
// //     qDesired_total.back().velocity.setZero();
// //     qDesired_total.back().acceleration.setZero();

// //     // 计算当前误差
// //     current_state.e_q = qDesired_total.back().position - current_state.q;
// //     current_state.e_qdot = qDesired_total.back().velocity - current_state.qdot;

// //     // 计算当前关节期望力矩（前馈项）
// //     qDesired_total.back().torque =
// //         pinocchio::rnea(panda_model, panda_data, current_state.q, current_state.qdot,
// //                         qDesired_total.back().acceleration);

// //     // 期望力矩限幅
// //     qDesired_total.back().torque = qDesired_total.back()
// //                                        .torque.array()
// //                                        .max(0.5 * joint_torque_min_limit.array())
// //                                        .min(0.5 * joint_torque_max_limit.array());

// //     // 计算惯性矩阵（返回上三角）
// //     pinocchio::crba(panda_model, panda_data, current_state.q, pinocchio::Convention::LOCAL);
// //     // 补全对称矩阵
// //     panda_data.M.triangularView<Eigen::StrictlyLower>() =
// //         panda_data.M.transpose().triangularView<Eigen::StrictlyLower>();
// //     // 关节力矩控制（反馈项）
// //     current_state.e_torque = panda_data.M * (PD_control.Kp_matrix * current_state.e_q +
// //                                              PD_control.Kd_matrix * current_state.e_qdot);

// //     // 力矩补偿
// //     qDesired_total.back().torque += current_state.e_torque;

// //     // 期望力矩限幅
// //     qDesired_total.back().torque = qDesired_total.back()
// //                                        .torque.array()
// //                                        .max(0.8 * joint_torque_min_limit.array())
// //                                        .min(0.8 * joint_torque_max_limit.array());

// //     // 发送力矩指令到机械臂
// //     for (int i = 0; i < 7; i++) {
// //       effort_joint_handles_[i].setCommand(qDesired_total.back().torque[i]);
// //     }
// //   }

// //   current_step++;

// //   // 计算时间
// //   auto end = std::chrono::high_resolution_clock::now();
// //   double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

// //   max_time = std::max(max_time, duration);
// // }

// // // 关节轨迹插补定义
// // std::vector<PandaHocbfTestController::Joint_Waypoint> PandaHocbfTestController::Joint_Trajectory(
// //     PandaHocbfTestController::Vector7d& qstart,
// //     PandaHocbfTestController::Vector7d& qend,
// //     double Tf,
// //     int N,
// //     int method) {
// //   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
// //   traj.resize(N);
// //   for (int i = 0; i < N; i++) {
// //     double s = PandaHocbfTestController::QuinticTimeScaling(Tf, i * dt);
// //     traj[i].position = qstart + s * (qend - qstart);
// //   }
// //   return traj;
// // }

// // // 笛卡尔轨迹插补定义
// // std::vector<PandaHocbfTestController::Actuator_Waypoint>
// // PandaHocbfTestController::Cartesian_Trajectory(PandaHocbfTestController::Actuator_Action& action)
// // {
// //   std::vector<PandaHocbfTestController::Actuator_Waypoint> traj{};
// //   traj.resize(action.num_waypoints);

// //   for (int i = 0; i < action.num_waypoints; i++) {
// //     double s = PandaHocbfTestController::QuinticTimeScaling(action.Tf_duration, i * dt);
// //     traj[i].R = action.Rstart *
// //                 pinocchio::exp3((pinocchio::log3(action.Rstart.transpose() * action.Rend) * s));
// //     traj[i].p = action.pstart + s * (action.pend - action.pstart);
// //     traj[i].X = pinocchio::SE3(traj[i].R, traj[i].p);
// //   }

// //   return traj;
// // }

// // // 逆运动学迭代求解
// // std::vector<PandaHocbfTestController::Joint_Waypoint>
// // PandaHocbfTestController::Inverse_Kinematics(
// //     PandaHocbfTestController::Actuator_Action& action,
// //     PandaHocbfTestController::Vector7d& q_init) {
// //   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
// //   traj.resize(action.num_waypoints);
// //   traj[0].position = q_init;
// //   // 当前位姿的Jacobian矩阵panda_model.nv=7
// //   pinocchio::Data::Matrix6x J(6, panda_model.nv);
// //   // se3 error
// //   PandaHocbfTestController::Vector6d err;
// //   // 目标关节速度
// //   Eigen::VectorXd v(panda_model.nv);
// //   // 修正jacobian
// //   pinocchio::Data::Matrix6 Jlog;
// //   // 正则化的jacobian
// //   pinocchio::Data::Matrix6 JJt;

// //   for (int i = 0; i < action.num_waypoints - 1; i++) {
// //     // 初始为0
// //     J.setZero();
// //     Jlog.setZero();
// //     JJt.setZero();
// //     // 目标位姿
// //     pinocchio::SE3 oMdes = action.X_desired[i + 1].X;
// //     // 坐标变换到当前参考系
// //     pinocchio::SE3 iMd = action.X_desired[i].X.actInv(oMdes);
// //     // 计算se3 error
// //     err = pinocchio::log6(iMd).toVector();
// //     // 用当前的q计算jacobian，End_effector_joint_id wrt base
// //     pinocchio::computeFrameJacobian(panda_model, panda_data, traj[i].position,
// //                                     End_effector_joint_id, pinocchio::LOCAL, J);

// //     // 正则化jacobian，解决奇异值问题
// //     pinocchio::Jlog6(iMd.inverse(), Jlog);
// //     J = -Jlog * J;
// //     JJt.noalias() = J * J.transpose();
// //     JJt.diagonal().array() += J_damping;
// //     v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
// //     traj[i + 1].position = pinocchio::integrate(panda_model, traj[i].position, v);
// //   }
// //   return traj;
// // }

// // // 五次样条插值,返回平滑系数
// // double PandaHocbfTestController::QuinticTimeScaling(double total_time, double current_time) {
// //   if (total_time <= 0.0)
// //     return 0.0;
// //   double t = std::max(0.0, std::min(current_time / total_time, 1.0));  // 限制在[0,1]区间
// //   double s = 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
// //   return s;
// // }

// // // 静止姿态动作
// // std::vector<PandaHocbfTestController::Joint_Waypoint> PandaHocbfTestController::Joint_Stay(
// //     PandaHocbfTestController::Vector7d& qstay,
// //     int N,
// //     double Tf_duration) {
// //   std::vector<PandaHocbfTestController::Joint_Waypoint> traj{};
// //   traj.resize(N);
// //   for (int i = 0; i < N; i++) {
// //     traj[i].position = qstay;
// //   }
// //   return traj;
// // }

// // // 初始化优化问题
// // void PandaHocbfTestController::casadi_opt_init() {
// //   casadi_cbf.opti = casadi::Opti();
// //   casadi_cbf.tau = casadi_cbf.opti.variable(7, 1);
// //   casadi_cbf.H = casadi_cbf.opti.parameter(7, 7);  // 参数占位符
// //   casadi_cbf.f = casadi_cbf.opti.parameter(7, 1);
// //   casadi_cbf.A = casadi_cbf.opti.parameter(1, 7);  // 参数占位符
// //   casadi_cbf.b = casadi_cbf.opti.parameter(1, 1);
// //   casadi_cbf.opti.minimize(
// //       0.5 *
// //           casadi::MX::mtimes(casadi::MX::mtimes(casadi_cbf.tau.T(), casadi_cbf.H),
// //           casadi_cbf.tau) +
// //       casadi::MX::mtimes(casadi_cbf.f.T(), casadi_cbf.tau));  // 添加目标函数

// //   // 约束1：线性约束
// //   casadi_cbf.opti.subject_to(casadi::MX::mtimes(casadi_cbf.A, casadi_cbf.tau) <= casadi_cbf.b);

// //   // 约束2：上下界
// //   casadi_cbf.opti.subject_to(cbf_params.lbU <= casadi_cbf.tau <= cbf_params.ubU);
// //   casadi_cbf.opti.solver("ipopt", casadiOptions, solverOptions);
// // }

// // // 优化问题求解
// // void PandaHocbfTestController::thread_task() {
// //   while (true) {
// //     if (casadi_opt_startup) {
// //       {
// //         std::lock_guard<std::mutex> lock(opt_mutex_);
// //         casadi_cbf.opti.set_value(casadi_cbf.H, cbf_params.H);
// //         casadi_cbf.opti.set_value(casadi_cbf.f, cbf_params.f);
// //         casadi_cbf.opti.set_value(casadi_cbf.A, cbf_params.A);
// //         casadi_cbf.opti.set_value(casadi_cbf.b, cbf_params.b);
// //       }

// //       auto solution = casadi_cbf.opti.solve();
// //       casadi::DM output = solution.value(casadi_cbf.tau);

// //       {
// //         std::lock_guard<std::mutex> lock(opt_mutex_);
// //         cbf_params.output_vec = std::vector<double>(output);
// //       }

// //       // 优化成功，更新step
// //       opt_step_current.store(opt_step_last.load() + 1, std::memory_order_release);
// //     }

// //     else {
// //       casadi_cbf.opti.set_value(casadi_cbf.H, 2 * casadi::DM::eye(7));
// //       casadi_cbf.opti.set_value(casadi_cbf.f, casadi::DM::zeros(7, 1));
// //       casadi_cbf.opti.set_value(casadi_cbf.A, casadi::DM::zeros(1, 7));
// //       casadi_cbf.opti.set_value(casadi_cbf.b, casadi::DM::zeros(1, 1));
// //       auto solution = casadi_cbf.opti.solve();
// //     }
// //   }
// // }

// // }  // namespace franka_example_controllers

// // PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PandaHocbfTestController,
// //                        controller_interface::ControllerBase)
