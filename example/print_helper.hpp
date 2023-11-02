/**
 * @file print_helper.hpp
 * @brief 打印接口调用结果
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
#define LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_

#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <iterator>
#include "rokae/robot.h"
#include "rokae/data_types.h"

inline std::ostream &operator<<(std::ostream &os, rokae::OperationState st) {
  using OP = rokae::OperationState;
  switch(st) {
    case OP::idle: os << "空闲"; break;
    case OP::jog: os << "Jog状态"; break;
    case OP::rtControlling: os << "实时模式控制中"; break;
    case OP::drag: os << "拖动已开启"; break;
    case OP::rlProgram: os << "RL工程运行中"; break;
    case OP::demo: os << "Demo演示中"; break;
    case OP::dynamicIdentify: os << "动力学辨识中"; break;
    case OP::frictionIdentify: os << "摩擦力辨识中"; break;
    case OP::loadIdentify: os << "负载辨识中"; break;
    case OP::moving: os << "运动中"; break;
    case OP::jogging: os << "Jog运动中"; break;
    case OP::unknown: default: os << "未知"; break;
  }
  return os;
}

template <class T, size_t S>
inline std::ostream &operator<<(std::ostream &os, const std::array<T,S> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

template <class T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Info &info) {
  os << "控制器版本 " << info.version << " | 机型 " << info.type << " | 轴数 " << info.joint_num;
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Frame &frame) {
  os << "[ X: " << frame.trans[0] << " Y: " << frame.trans[1] << " Z: " << frame.trans[2] <<
     " A: " << frame.rpy[0] << " B: " << frame.rpy[1] << " C: " << frame.rpy[2] << " ]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::CartesianPosition &cart) {
  os << "位姿 - [ X: " << cart.trans[0] << " Y: " << cart.trans[1] << " Z: " << cart.trans[2] <<
     " Rx: " << cart.rpy[0] << " Ry: " << cart.rpy[1] << " Rz: " << cart.rpy[2] << " ]";
  os << "\n臂角 - " << cart.elbow;
  if(!cart.confData.empty()) {
    os << "\nConf - [ ";
    for(const auto &d: cart.confData){
      os << d << " ";
    }
    os << "]";
  }
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Load &load) {
  os << "质量: " << load.mass << "kg, 重心 X: " << load.cog[0] << " Y: " << load.cog[1] << " Z: " << load.cog[2] <<
     ", 惯量 ix: " << load.inertia[0] << " iy: " << load.inertia[1] << " iz: " << load.inertia[2];
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const rokae::Toolset &toolset) {
  os << "手持 - " << toolset.end << "\n外部 - " << toolset.ref <<
     "\n负载 - " << toolset.load;
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const std::error_code &ec){
  if(ec) {
    os << ec.message();
  }
  return os;
}

template <typename... Args>
void print(std::ostream &os, Args&&... args) {
  ((os << ' '<< std::forward<Args>(args)), ...) << std::endl;
}

#endif //LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
