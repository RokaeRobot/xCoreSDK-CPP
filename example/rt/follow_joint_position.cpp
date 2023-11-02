/**
 * @file follow_joint_position.cpp
 * @brief 实时模式 - 点位跟随功能
 * 此功能需要使用xMateModel模型库，请设置编译选项XCORE_USE_XMATE_MODEL=ON
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "../print_helper.hpp"

std::atomic_bool running = true;
std::ostream &os = std::cout;
std::array<double, 7u> q_drag_xm7p = { 0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };
std::array<double, 6u> q_drag_er3 = { 0, M_PI/6, M_PI/3, 0, M_PI/2, 0 };
std::array<double, 6u> q_drag_cr7 = { 0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0 };
std::array<double, 6u> q_drag_sr3 = { 0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0 };

std::vector<std::array<double, 6>> points_xMateSR3();
std::vector<std::array<double, 6>> points_xMateER3();
std::vector<std::array<double, 6>> points_xMateCR();
std::vector<std::array<double, 7>> points_xMateERPro();

void example_followPosition(rokae::xMateRobot &robot,
                   const std::array<double, 6> &start_jnt,
                            const  std::vector<std::array<double, 6>> &points_list) {
  auto rtCon = robot.getRtMotionController().lock();
  std::thread updater;
  error_code ec;
  try {
    rtCon->MoveJ(0.3, robot.jointPos(ec), start_jnt);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto model = robot.model();
    rokae::FollowPosition follow_pose(robot, model);

    print(os, "开始跟随");
    Eigen::Transform<double, 3, Eigen::Isometry> bMe_desire = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    // 四元数0, 0, 1, 0, 转换成欧拉角是 A - 180, B - 0, C - 180
    bMe_desire.rotate(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));
    // X - 0.464, Y - 0.136, Z - 0.364
    bMe_desire.pretranslate(Eigen::Vector3d(0.464, 0.136, 0.364));
    follow_pose.start(bMe_desire);

    updater = std::thread([&]() {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      follow_pose.setScale(2);
      auto it = points_list.begin();
      while(running) {
        // 模拟每600ms更新一次位置
        while (running) {
          follow_pose.update(*it++);
          std::this_thread::sleep_for(std::chrono::milliseconds (600));
          if (it == points_list.end()) {
            it--;
            break;
          }
        }
        while (running) {
          follow_pose.update(*it--);
          std::this_thread::sleep_for(std::chrono::milliseconds (600));
          if (it == points_list.begin()) {
            break;
          }
        }
      }

    });
    std::thread consoleInput([]{
      while(getchar() != 'q'); // press 'q' to stop
      running = false;
    });
    consoleInput.detach();
    while(running);

    follow_pose.stop();
    updater.join();
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
    if(updater.joinable()) {
      running = false;
      updater.join();
    }
  }
}

int main() {
  using namespace rokae;
  using namespace std;
  using namespace rokae::RtSupportedFields;

  string remoteIP = "192.168.0.160";
  string localIP = "192.168.0.100";
  error_code ec;
  std::thread updater;
  xMateRobot robot;

  try {
    robot.connectToRobot(remoteIP, localIP);
  } catch (const std::exception &e) {
    std::cerr << e.what();
    return 0;
  }
  std::string robot_name = robot.robotInfo(ec).type;
  robot.setRtNetworkTolerance(20, ec);
  robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  try {
    auto rtCon = robot.getRtMotionController().lock();
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m});
  } catch (const std::exception &e) {
    std::cerr << e.what();
    return 0;
  }

  // 根据机型不同，适用的点位也不同，下方列出本示例测试过的机型
  // 其它机型建议重新适配点位，确认机械臂运动在安全的区域
  if(robot_name.find("CR7") != std::string::npos || robot_name.find("CR12") != std::string::npos ) {
    print(std::cout, "Run example for", robot_name);
    example_followPosition(robot, q_drag_cr7, points_xMateCR());
  }
  else if(robot_name.find("SR3") != std::string::npos || robot_name.find("SR4") != std::string::npos) {
    print(std::cout, "Run example for", robot_name);
    example_followPosition(robot, q_drag_sr3, points_xMateSR3());
  }
  else if(robot_name.find("xMate3") != std::string::npos ){
    print(std::cout, "Run example for", robot_name);
    example_followPosition(robot, q_drag_er3, points_xMateER3());
  }
  else {
    print(std::cerr, "示例程序中的点位尚未在该机型上使用过");
  }

  return 0;
}


/**
 * @brief 适用于xMateCR7/12的轴角度
 */
std::vector<std::array<double, 6>> points_xMateCR() {
  return {
    {0.0,0.52359877559829882,-1.5707963267948966,0.0,-1.0471975511965976,0.0},
    {0.035487243764309395,0.51814607597827012,-1.5806637828906964,2.2607167760800269e-06,-1.0427832278773319,0.035483868678714996},
    {0.035487413139973299,0.49909067450196087,-1.5511913155197452,9.3407434715976597e-07,-1.0913084154146433,0.035484721003813273},
    {0.071110628309228666,0.49472131619011839,-1.5588670555401505,2.9525157013044446e-06,-1.088002334582413,0.071104903418982859},
    {0.07111079887915675,0.47725271541391456,-1.5271895838122274,1.6430438164069251e-06,-1.1371457593165677,0.071105740122233752},
    {0.035487819996022588,0.48173318109035573,-1.5195061702471038,-7.7889329104740884e-08,-1.1403483431212154,0.035485819230280602},
    {-3.914398970759767e-07,0.49302452409585185,-1.5204379705599096,-1.4197211698704796e-06,-1.1281255631689342,1.198405110455232e-07},
    {-0.011784714972041753,0.50683103334891477,-1.5375423532367758,-1.2986068389168909e-06,-1.0972162124211495,-0.011783476036831626},
    {-7.7890503523341657e-07,0.51711158261127554,-1.5612157759668301,2.5489758866954378e-07,-1.0632642491857673,-9.773599037238323e-07}};
}

/**
 * @brief 适用于SR3 & SR4的轴角度
 */
std::vector<std::array<double, 6>> points_xMateSR3() {
  return {
    {-0.017938136876709584,0.49435159910311133,-1.6278070272570087,0.10223539561771972,-1.0233324615672779,-0.071357800901066984},
    {-0.032594083928095562,0.46634275144052351,-1.6842348590894223,0.20686587914942534,-1.0065436700890904,-0.14486251700897126},
    {-0.043703094256655285,0.43980306833212868,-1.7387524128620093,0.3128300518602628,-0.99763587014820621,-0.21887971635279299},
    {-0.051216089360091449,0.41501961247818264,-1.7900103133649683,0.41881018211089516,-0.99712229365660021,-0.29154392680357966},
    {-0.055306012301755358,0.39227789893146026,-1.8368733799718084,0.52341663484022405,-1.0050848002558492,-0.36101127048446552},
    {-0.056313209239857831,0.37179716947003955,-1.8786023105954095,0.62542150457539714,-1.0211378429141809,-0.42572952646181766},
    {-0.054650016827302569,0.35368739568221813,-1.9149211450368357,0.72395456855001505,-1.0444984061492966,-0.48462535901952425},
    {-0.05070577056919235,0.33794050853184859,-1.9459518831517613,0.81858624418967507,-1.0741362811783626,-0.53714380726237032},
    {-0.044783942543009984,0.32445262314869988,-1.9720809983080863,0.90929962091879346,-1.1089278543932601,-0.5831715234178394},
    {-0.037080277744188789,0.3130583447962762,-1.9938135626505962,0.99639141933330022,-1.1477775002697621,-0.62289965112758638}
  };
}

/**
 * @brief 适用于xMateER3/7 Pro的轴角度
 */
std::vector<std::array<double, (unsigned short)7>> points_xMateERPro() {
  return {
    {1.1984224905356572e-06,0.52361854956939269, 0, 1.0472093756318377,8.9881686790174287e-07,1.5708220928784431,4.4940843395087144e-06},
    {0.053236563559073066,0.52595866921924528, 0, 1.043601404881831,4.7936899621426287e-06,1.5720861290003356,0.053240518353291834},
    {0.053236803243571176,0.52990008105353537, 0, 0.95548571347371014,3.5952674716069715e-06,1.6562833983122764,0.053240218747669196},
    {0.10617160401952735,0.53732490759364893, 0, 0.94396623681018321,7.4901405658478573e-06,1.6603652253150412,0.10617603818274235},
    {0.10617136433502926,0.55044104254131654, 0, 0.84072621388700408,7.4901405658478573e-06,1.7504871958145678,0.10617573857711973},
    {0.053233687345095787,0.54245395611605984, 0, 0.85322959541526078,4.1944787168748001e-06,1.745959555645324,0.053239919142046566},
    {0.017760860994236547,0.53488232282085557, 0, 0.89270683067599588,1.7976337358034857e-06,1.7140503584123217,0.017765414999700583},
    {1.1984224905356572e-06,0.52740156989785025, 0, 0.95930316847506236,8.9881686790174287e-07,1.6549357722216693,5.6925068300443715e-06},
    {2.6365294791784457e-06,0.52364731170916556, 0, 1.0472302281831731,1.4980281131695715e-06,1.5708358747370843,1.2883041773258315e-05}};
}

/**
 * @brief 适用于xMateER3/7的轴角度
 */
std::vector<std::array<double, 6>> points_xMateER3() {
  return {
    {1.1984224905356572e-06,0.52361854956939269,1.0472093756318377,8.9881686790174287e-07,1.5708220928784431,4.4940843395087144e-06},
    {0.053236563559073066,0.52595866921924528,1.043601404881831,4.7936899621426287e-06,1.5720861290003356,0.053240518353291834},
    {0.053236803243571176,0.52990008105353537,0.95548571347371014,3.5952674716069715e-06,1.6562833983122764,0.053240218747669196},
    {0.10617160401952735,0.53732490759364893,0.94396623681018321,7.4901405658478573e-06,1.6603652253150412,0.10617603818274235},
    {0.10617136433502926,0.55044104254131654,0.84072621388700408,7.4901405658478573e-06,1.7504871958145678,0.10617573857711973},
    {0.053233687345095787,0.54245395611605984,0.85322959541526078,4.1944787168748001e-06,1.745959555645324,0.053239919142046566},
    {0.017760860994236547,0.53488232282085557,0.89270683067599588,1.7976337358034857e-06,1.7140503584123217,0.017765414999700583},
    {1.1984224905356572e-06,0.52740156989785025,0.95930316847506236,8.9881686790174287e-07,1.6549357722216693,5.6925068300443715e-06},
    {2.6365294791784457e-06,0.52364731170916556,1.0472302281831731,1.4980281131695715e-06,1.5708358747370843,1.2883041773258315e-05}};
}
