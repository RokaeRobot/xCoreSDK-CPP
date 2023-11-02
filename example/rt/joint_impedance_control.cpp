/**
 * @file joint_impedance_control.cpp
 * @brief 实时模式 - 轴空间阻抗控制
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

    // 设置要接收数据。其中jointPos_m是本示例程序会用到的
    robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m});

    static bool init = true;
    double time = 0;

    std::array<double, 7> jntPos{};
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, jntPos, q_drag_xm7p);

    // 设置轴空间阻抗系数，开始轴空间阻抗运动
    rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
    rtCon->startMove(RtControllerMode::jointImpedance);

    // 清除缓存数据
    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));

    std::function<JointPosition(void)> callback = [&, rtCon] {
      if(init) {
        robot.updateRobotState(std::chrono::milliseconds(1));
        robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
        init = false;
      }
      time += 0.001;
      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

      JointPosition cmd(7);
      for(unsigned i = 0; i < cmd.joints.size(); ++i) {
        cmd.joints[i] = jntPos[i] + delta_angle;
      }

      if(time > 60) {
        cmd.setFinished(); // 60秒后结束
      }
      return cmd;
    };

    rtCon->setControlLoop(callback);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
  } catch (const std::exception &e) {
    std::cout << e.what();
  }
  return 0;
}
