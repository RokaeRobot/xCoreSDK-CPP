/**
 * @file cartesian_s_line.cpp
 * @brief 实时模式 - 笛卡尔空间S规划
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"

using namespace rokae;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.100"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // 设置要接收数据
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});
    std::array<double, 16> init_pos{}, end_pos{};
    std::array<double,7> jntPos{}, delta{};
    Eigen::Quaterniond rot_cur;
    Eigen::Matrix3d mat_cur;
    double delta_s;

    robot.updateRobotState(std::chrono::milliseconds(1));
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, jntPos, q_drag_xm7p);

    static bool init = true;
    double time = 0;

    //开始运动前先设置为笛卡尔空间位置控制
    rtCon->startMove(RtControllerMode::cartesianPosition);

    std::function<CartesianPosition()> callback = [&, rtCon]() {
      time += 0.001; // 按1ms为周期规划
      if(init) {
        // 读取当前位置
        robot.getStateData(RtSupportedFields::tcpPose_m, init_pos);
        end_pos = init_pos;
        end_pos[11] -= 0.2;
        init = false;
      }

      std::array<double, 16> pose_start = init_pos;

      Eigen::Vector3d pos_1(pose_start[3], pose_start[7], pose_start[11]);
      Eigen::Vector3d pos_2(end_pos[3], end_pos[7], end_pos[11]);

      Eigen::Vector3d pos_delta = pos_2 - pos_1;
      double s = pos_delta.norm();
      Eigen::Vector3d pos_delta_vector = pos_delta.normalized();

      CartMotionGenerator cart_s(0.05, s);
      cart_s.calculateSynchronizedValues(0);

      Eigen::Matrix3d mat_start, mat_end;
      mat_start << pose_start[0], pose_start[1], pose_start[2], pose_start[4], pose_start[5], pose_start[6],
        pose_start[8], pose_start[9], pose_start[10];

      mat_end << end_pos[0], end_pos[1], end_pos[2], end_pos[4], end_pos[5], end_pos[6], end_pos[8],
        end_pos[9], end_pos[10];

      Eigen::Quaterniond rot_start(mat_start);
      Eigen::Quaterniond rot_end(mat_end);

      Eigen::Vector3d pos_cur;
      CartesianPosition cmd;
      if (!cart_s.calculateDesiredValues(time, &delta_s)) {
        pos_cur = pos_1 + pos_delta * delta_s / s;
        Eigen::Quaterniond rot_cur = rot_start.slerp(delta_s / s, rot_end);
        mat_cur = rot_cur.normalized().toRotationMatrix();
        std::array<double, 16> new_pose = {
          {mat_cur(0, 0), mat_cur(0, 1), mat_cur(0, 2), pos_cur(0), mat_cur(1, 0), mat_cur(1, 1),
           mat_cur(1, 2), pos_cur(1), mat_cur(2, 0), mat_cur(2, 1), mat_cur(2, 2), pos_cur(2), 0, 0, 0, 1}};
        cmd.pos = new_pose;
      } else {
        cmd.setFinished();
      }
      return cmd;
    };

    rtCon->setControlLoop(callback);
    rtCon->startLoop(true);
    print(std::cout, "控制结束");

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }

  return 0;
}
