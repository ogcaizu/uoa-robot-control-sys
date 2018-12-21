/*****************************************************************************
** Ifdefs
*****************************************************************************/

//#ifndef NAVIGATION_ROBOT_HPP_
#define NAVIGATION_ROBOT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <kobuki_driver/kobuki.hpp>
#include <service_robot_msg/RobotState.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <sstream>
#include <istream>
using namespace std;

/*****************************************************************************
** Define
*****************************************************************************/
#define EAST (-(ecl::pi) / 2)
#define WEST ((ecl::pi) / 2)
#define NOUTH (0)
#define SOUTH (ecl::pi)
#define ANGLE_TOLERANCE 0.1
#define DISTANCE_TOLERANCE 0.001
#define LINEAR_PARAM 0.3
#define ANGULAR_PARAM 0.52

/*****************************************************************************
** global valiable
*****************************************************************************/
double linear_param = LINEAR_PARAM;
double angular_param = ANGULAR_PARAM;
bool shutdown_req = false;

/*****************************************************************************
** Classes
*****************************************************************************/

/**
* @brief Keyboard remote control for our robot core (mobile base).
*
*/

class PathPoint
{
private:
  std::string m_direction;
  double m_distance;

public:
  PathPoint(string str = "None", double dis = -1)
  {
    m_direction = str;
    m_distance = dis;
  };
  PathPoint(const PathPoint& x)
      : m_direction(x.m_direction), m_distance(x.m_distance){};
  void print()
  {
    cout << "direction: " << m_direction << "   distance: " << m_distance
         << endl;
  }
  double getDistance() { return m_distance; }
  std::string getDirection() { return m_direction; }
  void setDirection(std::string str) { m_direction = str; }
};

class Route
{
private:
  std::string m_room_name;
  std::vector<PathPoint> m_path_list;
  std::vector<PathPoint>::iterator itr = m_path_list.begin();
  std::vector<PathPoint> return_path_list;
  std::vector<PathPoint>::iterator itr_rev = return_path_list.begin();

public:
  Route(string str, std::vector<PathPoint> list)
  {
    m_room_name = str;
    m_path_list = list;
  };
  Route(const Route& x)
      : m_room_name(x.m_room_name), m_path_list(x.m_path_list){};

  std::vector<PathPoint>& get_route() { return m_path_list; }
  std::vector<PathPoint>& get_return_route()
  {
    return_path_list = m_path_list;
    std::reverse(return_path_list.begin(), return_path_list.end());
    reverse_path_list();
    return return_path_list;
  }

  void reverse_path_list()
  {
    for (itr_rev = return_path_list.begin(); itr_rev != return_path_list.end();
         ++itr_rev)
    {
      PathPoint pp = *itr_rev;
      std::string direction = pp.getDirection();
      char c = direction[0];
      switch (c)
      {
      case 'N':
        pp.setDirection("South");
        break;
      case 'S':
        pp.setDirection("Nouth");
        break;
      case 'E':
        pp.setDirection("West");
        break;
      case 'W':
        pp.setDirection("East");
        break;
      }
      *itr_rev = pp;
    }
  }

  std::string get_room_name() { return m_room_name; }

  void print_all()
  {
    cout << "\x1b[36m" << m_room_name << "\x1b[39m" << endl;
    // cout << "\troute_list " << endl;
    print_path_list(m_path_list, itr);
  }
  void print_path_list(std::vector<PathPoint>& path_list,
                       std::vector<PathPoint>::iterator itr)
  {
    for (itr = path_list.begin(); itr != path_list.end(); ++itr)
    {
      PathPoint PathPoint = *itr;
      PathPoint.print();
    }
  }
};

class KobukiManager
{
public:
  /*********************
  ** C&D
  **********************/
  KobukiManager();
  ~KobukiManager();
  bool init(ros::NodeHandle& nh);

  /*********************
  ** Callbacks
  **********************/
  void processStreamData();

  /*********************
  ** Accessor
  **********************/
  ecl::LegacyPose2D<double> getPose();
  void req_input_keyboard();
  void print_req_all();
  void reset_Dx_Dth();
  void reset_Dx();
  double getDx();
  void init_itr();
  void set_path(std::vector<PathPoint>&);
  void set_return_path(std::vector<PathPoint>&);
  void set_robot_mode_standby();
  void set_robot_mode_navi();

  void get_distination(std::string, KobukiManager&);
  void set_distination(KobukiManager&);

  void kobuki_start();
  // void kobuki_stop();

  // public Member variable
  bool navigation_finish = false;

private:
  // Private Member variable distruction vector
  std::vector<PathPoint> kobuki_req;
  std::vector<PathPoint>::iterator itr = kobuki_req.begin();
  std::vector<PathPoint> kobuki_req_home;
  std::vector<Route> distinations;
  std::vector<Route>::iterator dist_itr;

  double dx, dth;
  string r_mode;
  ecl::LegacyPose2D<double> pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;

  // publisher declaraion
  ros::Publisher robot_state_pub;
  service_robot_msg::RobotState pub_msg;

  double linear_vel;
  double angular_vel;

  // Keylogging
  int key_file_descriptor;
  struct termios original_terminal_state;

  // distruction input
  void KeyboardInput(std::vector<PathPoint>&);

  // PathPoint list print
  void print_req(std::vector<PathPoint>&, std::vector<PathPoint>::iterator);

  // controll robot
  void RobotControll(std::vector<PathPoint>&,
                     std::vector<PathPoint>::iterator&);

  // publish robot state
  void PublishRobotState();
};
