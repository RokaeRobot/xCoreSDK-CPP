/**
 * @file rt_industrial.cpp
 * @brief 实时模式 - 工业6轴机型支持位置控制
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <cmath>
#include <thread>
#include <iterator>
#include "../print_helper.hpp"
#include "Eigen/Geometry"
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

std::ostream &os = std::cout;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::StandardRobot robot(ip, "192.168.0.180");
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);

    robot.setPowerState(true, ec);
    auto rtCon = robot.getRtMotionController().lock();

    // 重置末端坐标系，与法兰重合
    std::array<double, 16> _end{};
    Utils::postureToTransArray({0,0,0,0,0,0}, _end);
    rtCon->setEndEffectorFrame(_end, ec);

    // 示例程序使用机型: XB7h-R707
    // ***** 1. 从当前位置MoveJ运动到发货位置 *****
    rtCon->MoveJ(0.4, robot.jointPos(ec), Utils::degToRad(std::array<double, 6>({0, -15, 60, 0, 45, 0})));

    // ***** 2. 圆弧运动 (X-Y平面上) *****
    CartesianPosition start, aux, target;
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::endInRef, ec), start.pos);
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_aux, trans_end;
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
    trans_end = trans_start;
    trans_aux = trans_start;
    // 辅助点X+0.28, Y-0.05; 目标点Y-0.15
    trans_aux[0] += 0.28;
    trans_aux[1] -= 0.05;
    trans_end[1] -= 0.15;

    Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);
    rtCon->MoveC(0.2, start, aux, target);

#if 0
    // ***** 示例：设置安全区域 *****
    // 以当前位姿为安全区域中心点, X方向长度1m, Y方向长度0.8m, Z方向长度0.1m
    // 若设置成功, 后面的MoveL的目标点的Z值超出安全区域，机器人会下电处理
    std::array<double, 16> _centre{};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), _centre);
    rtCon->setCartesianLimit({1, 0.8, 0.1}, _centre, ec);
#endif

    // ***** 3. 直线运动 *****
    auto _pose_start = robot.posture(rokae::CoordinateType::endInRef, ec);
    auto _pose_target = _pose_start;
    // 沿Z+0.2m, 绕Ry+60°
    _pose_target[2] += 0.2;
    _pose_target[4] += Utils::degToRad(60);
    Utils::postureToTransArray(_pose_start, start.pos);
    Utils::postureToTransArray(_pose_target, target.pos);

    print(os, "MoveL start position:", start.pos, "Target:", target.pos);

    rtCon->MoveL(0.3, start, target);

    // ***** 4. 设置实时模式末端坐标系 *****
    Utils::postureToTransArray(std::array<double, 6>({0.1, 0, 0, 0, M_PI_2, 0}), _end);
    rtCon->setEndEffectorFrame(_end, ec);
    rtCon->MoveJ(0.4, robot.jointPos(ec), Utils::degToRad(std::array<double, 6>({0, -15, 60, 0, 45, 0})));

    // 说明：实时模式的工具坐标系设置是独立的，因此不能用robot.posture(CoordinateType::endInRef)接口来获取末端位姿；
    // 下方示例直接给出设置末端坐标后的起始位姿;
    // 或者，可以接收实时状态数据，通过robot.getStateData(RtSupportedFields::tcpPose_m, start.pos)获取
    Utils::postureToTransArray({0.1036, 0,0.415, 0.042, -M_PI_2, -0.0424}, start.pos);
    target.pos = start.pos;
    target.pos[3] += 0.35;
    print(os, "MoveL start position:", start.pos, "Target:", target.pos);
    rtCon->MoveL(0.3, start, target);

    // ***** 5. 关闭实时模式 *****
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}