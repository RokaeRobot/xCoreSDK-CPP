/**
 * @file cartesian_impedance_control.cpp
 * @brief 实时模式 - 笛卡尔阻抗控制
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.100"); // ****   xMate 7-axis
    robot.setRtNetworkTolerance(50, ec);
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    std::array<double,7> q_drag_xm7p = {0,M_PI/6,0,M_PI/3,0,M_PI/2,0};
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag_xm7p);

    // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
    std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
    rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);
    // 设置笛卡尔阻抗系数
    rtCon->setCartesianImpedance({1200, 1200, 0, 100, 100, 0}, ec);
    // 设置X和Z方向3N的期望力
    rtCon->setCartesianImpedanceDesiredTorque({3, 0, 3, 0, 0, 0}, ec);

    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);

    rtCon->startMove(RtControllerMode::cartesianImpedance);

    double time = 0;
    std::atomic<bool> stopManually {true};
    std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
      time += 0.001;
      constexpr double kRadius = 0.2;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
      double delta_z = kRadius * (std::cos(angle) - 1);

      CartesianPosition output{};
      output.pos = init_position;
      output.pos[7] += delta_z;

      if(time > 40){
        std::cout << "运动结束" <<std::endl;
        output.setFinished();
        stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
      }
      return output;
    };
    rtCon->setControlLoop(callback);
    rtCon->startLoop(false);
    while(stopManually.load());
    rtCon->stopLoop();
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
  return 0;
}
