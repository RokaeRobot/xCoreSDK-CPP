/**
 * @file xmatemodel_er3_er7p.cpp
 * @brief xMate运动学和动力学计算库，以xMate3和xMateEr7Pro为例
 * 此示例需要使用xMateModel模型库，请设置编译选项XCORE_USE_XMATE_MODEL=ON
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"

using namespace std;
using namespace rokae;

ostream &os = std::cout; // print to console

void xMateErPro7_model(xMateModel<7> &model) {
  try {
    std::array<double, 7> zeros = {0, 0, 0, 0, 0, 0, 0},
      jointPos_in = Utils::degToRad(array<double,7>({5, 46, -10, 91, 1, -105, 11})),
      jointVel_in = {0.3, 0.2, 0.5, 0.4, 0.1, 0.1, 0.1},
      jointAcc_in = {1.3, 3.1, 4.1, 1.5, 1.6, 4.1, 8.1},
      jointInit = Utils::degToRad(array<double,7>({6, 45, -9, 92, 0, -103, 10})),
      array1 {}, array2 {}, array3 {}, array4 {};

    std::array<double, 16> F_TO_EE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
      EE_TO_K = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    print(os, "Jacobian -", model.jacobian(zeros));
    print(os, "Jacobian given EE -", model.jacobian(zeros, F_TO_EE, EE_TO_K, SegmentFrame::endEffector));

    model.getTorqueNoFriction(zeros, zeros, zeros, array1, array2, array3, array4);
    print(os, "TorqueNoFriction full -", array1);
    print(os, "TorqueNoFriction inertia -", array2);
    print(os, "TorqueNoFriction coriolis -", array3);
    print(os, "TorqueNoFriction gravity -", array4);

    print(os, "Torque inertia -", model.getTorque(zeros, zeros, zeros, TorqueType::inertia));

    auto pos = model.getCartPose(jointPos_in);
    print(os, "Flange posture -", pos);

    model.setTcpCoor(F_TO_EE, EE_TO_K); // 设置末端执行器坐标
    auto pos_e = model.getCartPose(jointPos_in, SegmentFrame::endEffector);
    print(os, "EE posture -", pos_e);

    auto cartVel_out = model.getCartVel(jointPos_in, jointVel_in);
    auto cartAcc_out = model.getCartAcc(jointPos_in, jointVel_in, jointAcc_in);
    print(os, "Cartesian velocity -", cartVel_out);
    print(os, "Cartesian acceleration -", cartAcc_out);

    print(os, "Joint velocities -", model.getJointVel(cartVel_out, jointPos_in));
    print(os, "Joint accelerations -", model.getJointAcc(cartAcc_out, jointPos_in, jointVel_in));

    double psi = Utils::degToRad(-7.543);
    array<int, 8> confData = {0, 0, -1, 1, 0, -2, 0, 1};
    auto ret = model.getJointPos(pos, psi, jointInit, array1);
    print(os, "IK calculation -", array1, "| ret:", ret);

    pos = model.getCartPose(array1);
    print(os, "FK calculation -", pos);

    // 设置负载
    double load_mass = 1.0;
    std::array<double, 3> load_centre = {0.1, 0.1, 0.1}, load_inertia = {3.0, 2.0, 5.0};
    model.setLoad(load_mass, load_centre, load_inertia);

  } catch (const std::exception &e) {
    print(cerr, e.what());
  }
}


void xMateEr3_model(xMateModel<6> &model) {
  try {
    std::array<double, 6> zeros {},
      jointPos_in = Utils::degToRad(array<double,6>({-20, 37, 70, 0, 71, -19})),
      jointVel_in = {0.3, 0.2, 0.5, 0.4, 0.1, 0.1},
      jointAcc_in = {1.3, 3.1, 4.1, 1.5, 1.6, 4.1},
      jointInit = Utils::degToRad(array<double,6>({-21, 38, 71, 0, 70, -20})),
      array1 {}, array2 {}, array3 {}, array4 {};
    std::array<double, 16> F_TO_EE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
      EE_TO_K = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    print(os, "Jacobian -", model.jacobian(zeros));
    print(os, "Jacobian given EE -", model.jacobian(zeros, F_TO_EE, EE_TO_K, SegmentFrame::endEffector));

    model.getTorqueNoFriction(zeros, zeros, zeros, array1, array2, array3, array4);
    print(os, "TorqueNoFriction full -", array1);
    print(os, "TorqueNoFriction inertia -", array2);
    print(os, "TorqueNoFriction coriolis -", array3);
    print(os, "TorqueNoFriction gravity -", array4);

    print(os, "Torque inertia -", model.getTorque(zeros, zeros, zeros, TorqueType::inertia));

    auto pos = model.getCartPose(jointPos_in);
    print(os, "Flange posture -", pos);

    model.setTcpCoor(F_TO_EE, EE_TO_K); // 设置末端执行器坐标
    auto pos_e = model.getCartPose(jointPos_in, SegmentFrame::endEffector);
    print(os, "EE posture -", pos_e);

    auto cartVel_out = model.getCartVel(jointPos_in, jointVel_in);
    auto cartAcc_out = model.getCartAcc(jointPos_in, jointVel_in, jointAcc_in);
    print(os, "Cartesian velocity -", cartVel_out);
    print(os, "Cartesian acceleration -", cartAcc_out);

    print(os, "Joint velocities -", model.getJointVel(cartVel_out, jointPos_in));
    print(os, "Joint accelerations -", model.getJointAcc(cartAcc_out, jointPos_in, jointVel_in));

    double psi = Utils::degToRad(-7.543);
    array<int, 8> confData = {0, 0, -1, 1, 0, -2, 0, 1};
    auto ret = model.getJointPos(pos, psi, jointInit, array1);
    print(os, "IK calculation -", array1, "| ret:", ret);

    pos = model.getCartPose(array1);
    print(os, "FK calculation -", pos);

    // 设置负载
    double load_mass = 2.0;
    std::array<double, 3> load_centre = {0.1, 0.1, 0.1}, load_inertia = {3.0, 2.0, 5.0};
    model.setLoad(load_mass, load_centre, load_inertia);

  } catch (const std::exception &e) {
    print(cerr, e.what());
  }
}

int main() {
  bool test_Er3 = false;
  try {
    std::string ip = "192.168.0.160";

    if(test_Er3) {
      rokae::xMateRobot robot(ip); // ****   xMate 6-axis
      auto model = robot.model(); // 返回xMateModel类
      xMateEr3_model(model);
    } else {
      rokae::xMateErProRobot robot(ip);
      auto model = robot.model(); // 返回xMateModel类
      xMateErPro7_model(model);
    }
  }  catch (const std::exception &e) {
    print(cerr, e.what());
  }

}