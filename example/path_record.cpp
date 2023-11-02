/**
 * @file path_record.cpp
 * @brief 协作机型拖动示教，路径录制和回放
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include <unordered_map>
#include "rokae/robot.h"

using namespace std;
using namespace rokae;

char parseInput(std::string &str);
void WaitRobot(BaseRobot *robot);
void printHelp();

int main() {
  try {
    std::string ip = "192.168.0.160";
    error_code ec;
    std::vector<std::string> paths;
    xMateRobot robot(ip); // xMate 6轴机型

    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);

    printHelp();

    char cmd = ' ';
    while(cmd != 'q') {
      std::string str;
      // 从控制台读取命令
      getline(std::cin, str);
      cmd = parseInput(str);

      switch(cmd) {
        case 'p':
          if(str == "on") { robot.setPowerState(true, ec); std::cout << "* 机器人上电\n"; }
          else { robot.setPowerState(false, ec); std::cout << "* 机器人下电\n"; }
          if(ec) break; continue;
        case 'm':
          if(str == "manual") { robot.setOperateMode(OperateMode::manual, ec); std::cout << "* 手动模式\n"; }
          else { robot.setOperateMode(OperateMode::automatic, ec); std::cout << "* 自动模式\n"; }
          if(ec) break; continue;
        case 'd':
          // 打开拖动前置条件: 需要切换机器人操作模式为手动模式，并下电
          if(str == "on") { robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec); cout << "* 打开拖动\n"; }
          else { robot.disableDrag(ec); std::cout << "* 关闭拖动\n"; }
          if(ec) break; continue;
        case 'a':
          robot.startRecordPath(30, ec); std::cout << "* 开始录制路径\n";
          if(ec) break; continue;
        case 'b':
          robot.stopRecordPath(ec); std::cout << "* 停止录制路径\n";
          if(ec) break; continue;
        case 's':
          robot.saveRecordPath(str, ec); std::cout << "* 保存路径为: " << str << endl;
          if(ec) break; continue;
        case 'c':
          robot.cancelRecordPath(ec); cout << "* 取消录制\n";
          if(ec) break; continue;
        case 'u':
          paths = robot.queryPathLists(ec);
          if(paths.empty()) cout << "* 没有已保存的路径\n";
          else {
            cout << "* 已保存的路径: ";
            for(auto p : paths) cout << p << ", ";
            cout << endl;
          }
          if(ec) break; continue;
        case 'v':
          cout << "* 删除路径\"" << str << "\"\n";
          robot.removePath(str, ec);
          if(ec) break; continue;
        case 'r': {
          robot.replayPath(str, 1.0, ec);
          if (ec) break;
          cout << "* 开始回放路径\"" << str << "\", 速率100%\n";
          WaitRobot(&robot);
          cout << "* 回放结束\n";
          continue;
        }
        case 'z':
          robot.moveReset(ec); cout << "* 重置运动缓存\n";
          if(ec) break; continue;
        case 'h':
          printHelp(); continue;
        case 'q':
          std::cout << " --- Quit --- \n"; continue;
        default:
          std::cerr << "无效输入\n"; continue;
      }
      cerr << "! 错误信息: " << ec.message() << endl;
    }

    robot.disconnectFromRobot(ec);
  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}

static const std::unordered_map<std::string, char> ConsoleInput = {
  {"quit", 'q'},
  {"power", 'p'},
  {"drag", 'd'},
  {"mode", 'm'},
  {"start", 'a'},
  {"stop", 'b'},
  {"save", 's'},
  {"cancel", 'c'},
  {"query", 'u'},
  {"remove", 'v'},
  {"reset", 'z'},
  {"replay", 'r'},
  {"help", 'h'}
};

void printHelp() {
  cout << " --- 拖动与路径回放使用示例 --- " << endl
       << "格式 <命令>[:参数] 例如 save:track0" << endl << endl
       << "命令                 |  参数"        << endl
       << "power   机器人上下电   | on|off"      << endl
       << "mode    手/自动模式    | manual|auto" << endl
       << "drag    打开关闭拖动   | on|off"      << endl
       << "start   开始录制路径   |"             << endl
       << "stop    结束录制路径   |"             << endl
       << "save    保存路径      | 路径名称"      << endl
       << "cancel  取消录制      |"             << endl
       << "query   查询已保存路径 |"            << endl
       << "remove  删除路径      | 路径名称"     << endl
       << "reset   重置运动缓存   |"             << endl
       << "replay  路径回放      | 路径名称"      << endl
       << "quit    结束\n";
}

char parseInput(std::string &str) {
  size_t delimiter;
  std::string cmd(str);
  if((delimiter = str.find(':')) != std::string::npos) {
    cmd = str.substr(0, delimiter);
    str = str.substr(delimiter + 1);
  }
  if(ConsoleInput.count(cmd))  return ConsoleInput.at(cmd);
  else return ' ';
}

/**
 * @brief 等待机器人运动结束
 */
void WaitRobot(BaseRobot *robot) {
  bool running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }
}