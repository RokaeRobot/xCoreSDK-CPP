/**
 * @file rl_project.cpp
 * @brief 加载和运行RL工程
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <unordered_map>
#include "rokae/robot.h"

using namespace std;
using namespace rokae;

void printHelp();
static const std::unordered_map<std::string, char> ConsoleInput = {
  {"on", '0'}, {"off", 'x'}, {"quit", 'q'},
  {"info", 'i'}, {"load", 'l'}, {"main", 'm'},
  {"start", 's'}, {"pause", 'p'}, {"tool", 't'},
  {"wobj", 'w'}, {"opt", 'o'}, {"help", 'h'}
};

int main() {
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateRobot robot(ip); // ****   xMate 6-axis

    robot.setMotionControlMode(MotionControlMode::NrtRLTask,ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    printHelp();
    char cmd = ' ';
    while(cmd != 'q') {
      std::string str;
      getline(cin, str);
      if(ConsoleInput.count(str)){
        cmd = ConsoleInput.at(str);
      } else {
        cmd = ' '; }

      switch(cmd) {
        case '0':
          robot.setPowerState(true, ec); cout << "* 机器人上电\n";
          if(ec) break; continue;
        case 'x':
          robot.setPowerState(false, ec); cout << "* 机器人下电\n";
          if(ec) break; continue;
        case 'i': {
          cout << "* 查询工程信息:\n";
          auto infos = robot.projectsInfo(ec);
          if(infos.empty()) { cout << "无工程\n"; }
          else {
            for(auto &info : infos) {
              cout << "名称: " << info.name << " 任务: ";
              for(auto &t: info.taskList) {
                cout << t << " ";}
              cout << endl; }
          }
          if(ec) break; } continue;
        case 'l':{
          cout << "* 加载工程, 请输入加载工程名称: ";
          std::string name, line, task;
          vector<string> tasks;
          getline(cin, name);
          cout << "请输入要运行的任务,空格分割: ";
          getline(cin, line);
          istringstream iss(line);
          while (iss >> task)
            tasks.push_back(task);
          robot.loadProject(name, tasks, ec);
          if(ec) break; } continue;
        case 'm':
          robot.ppToMain(ec);
          cout << "* 程序指针指向main\n";
          if(ec) break; continue;
        case 's':
          robot.runProject(ec); cout << "* 开始运行工程\n";
          if(ec) break; continue;
        case 'p':
          robot.pauseProject(ec); cout << "* 暂停运行\n";
          if(ec) break; continue;
        case 't': {
          cout << "* 查询工具信息\n";
          auto tools = robot.toolsInfo(ec);
          if(tools.empty()) cout << "无工具\n";
          else {
            for(auto &tool : tools) {
              cout << "工具: " << tool.name << ", 质量: " << tool.load.mass << endl;
            } }
          if(ec) break; } continue;
        case 'w': {
          cout << "* 查询工件信息\n";
          auto wobjs = robot.wobjsInfo(ec);
          if(wobjs.empty()) cout << "无工件\n";
          else {
            for(auto &wobj:wobjs) {
              cout << "工件: " << wobj.name << ", 是否手持: " << boolalpha << wobj.robotHeld << endl;}
          }
          if(ec) break; } continue;
        case 'o':{
          cout << "* 设置运行参数, 请依次输入运行速率和是否循环([0]单次/[1]循环), 空格分隔: ";
          double rate; bool isLoop; string line;
          getline(cin, line);
          istringstream iss(line);
          iss >> rate >> isLoop;
          robot.setProjectRunningOpt(rate, isLoop, ec);
          if(ec) break;} continue;
        case 'h':
          printHelp(); continue;
        case 'q':
          std::cout << " --- Quit --- \n"; continue;
        default:
          std::cerr << "无效输入\n"; continue;
      }
      cerr << "! 错误信息: " << ec.message() << endl;
    }

  } catch (const rokae::Exception &e) {
    std::cout << e.what();
  }
  return 0;
}

void printHelp() {
  cout << " --- 运行RL工程示例 --- \n\n"
  << "     命令   \n"
  << "on    机器人上电\n"
  << "off   机器人下电\n"
  << "info  查询工程列表\n"
  << "load  加载工程\n"
  << "main  程序指针指向main\n"
  << "start 开始运行\n"
  << "pause 暂停运行\n"
  << "opt   设置运行参数\n"
  << "tool  查询工具信息\n"
  << "wobj  查询工件信息\n"
  << "help  查看示例程序所有命令\n"
  << "quit  结束\n";
}