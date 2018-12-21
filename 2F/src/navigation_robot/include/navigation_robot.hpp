/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef NAVIGATION_ROBOT_HPP
#define NAVIGATION_ROBOT_HPP

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
#include <office_guide_robot/r_state.h>
#include <office_guide_robot/r_req.h>
#include <office_guide_robot/Position.h>
#include "../../kobuki/kobuki_core/kobuki_driver/include/kobuki_driver/modules/diff_drive.hpp"
#include <ecl/mobile_robot.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <sstream>
#include <istream>
#include <time.h>

#include <chrono>
#include <future>

#include "Eigen/Core"
using namespace std;
// using Eigen::MatrixXd;
/*****************************************************************************
** Define
*****************************************************************************/
#define EAST (-(ecl::pi) / 2)
#define WEST ((ecl::pi) / 2)
#define NOUTH (0)
#define SOUTH (ecl::pi)
#define ANGLE_TOLERANCE 0.3
#define DISTANCE_TOLERANCE 0.03
#define LINEAR_PARAM 0.33
#define ANGULAR_PARAM 1.50
#define FILENAME "/home/turtle2/ros/src/navigation_robot/src/dist.txt"
#define DIFF_STRAIGHT 0.4
#define ANGLE_CONST 0.5

/*****************************************************************************
** global valiable
*****************************************************************************/
extern double linear_param;
extern double angular_param;
extern bool shutdown_req;
extern bool msg_receive;
extern std::string receive_msg_goal;
extern double global_v;
extern double global_omega;
extern int global_QR_heading_n;
extern int global_kalman_heading_n;

/*****************************************************************************
** Classes
*****************************************************************************/
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
  std::string m_goal_point;
  std::string m_room_name;
  std::vector<PathPoint> m_path_list;
  std::vector<PathPoint>::iterator itr = m_path_list.begin();
  std::vector<PathPoint> return_path_list;
  std::vector<PathPoint>::iterator itr_rev = return_path_list.begin();

public:
  Route(string goal, string str, std::vector<PathPoint> list)
  {
    m_goal_point = goal;
    m_room_name = str;
    m_path_list = list;
  };
  Route(const Route& x)
      : m_goal_point(x.m_goal_point), m_room_name(x.m_room_name),
        m_path_list(x.m_path_list){};

  std::vector<PathPoint>& get_route() { return m_path_list; }
  std::vector<PathPoint>& get_return_route()
  {
    return_path_list = m_path_list;
    std::reverse(return_path_list.begin(), return_path_list.end());
    reverse_path_list();
    PathPoint front_face("Nouth", 0);
    return_path_list.push_back(front_face);
    return return_path_list;
  }

  std::string get_goal_point() { return m_goal_point; }
  void set_goal_point(std::string goal_point) { m_goal_point = goal_point; }

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
  ecl::LegacyPose2D<double> get_odomPose();
  ecl::LegacyPose2D<double> get_kalmanPose();
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
  string get_robot_mode();

  void set_QRpose(ecl::LegacyPose2D<double>);
  ecl::LegacyPose2D<double> get_QRpose();

  void get_distination(KobukiManager&);
  void set_distination(KobukiManager&);

  void kobuki_start();
  // void kobuki_stop();

  void PublishRobotState();

  void msgCallback(const office_guide_robot::r_req::ConstPtr&);
  void msgCallback1(const office_guide_robot::Position::ConstPtr&);

  // public Member variable
  bool navigation_finish = false;
  bool origin_ok = false;         // origin_variable
  bool turn_origin_ok = false;    // origin_variable
  bool forward_origin_ok = false; // origin_variable

private:
  // Private Member variable distruction vector
  struct timespec time;
  double prev_time;
  double delta_t;
  double kalman_dx = 0;

  double linear_param = LINEAR_PARAM;
  double angular_param = ANGULAR_PARAM;
  std::string input_method = "";

  std::vector<PathPoint> kobuki_req;
  std::vector<PathPoint>::iterator itr = kobuki_req.begin();
  std::vector<PathPoint> kobuki_req_home;
  std::vector<Route> distinations;
  std::vector<Route>::iterator dist_itr;

  string r_mode;

  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;

  // publisher declaration
  ros::Publisher robot_state_pub;
  office_guide_robot::r_state pub_msg;

  // subscribe message declaration
  // office_guide_robot::r_req sub_msg;

  // odometry data
  ecl::LegacyPose2D<double> odom_pose;

  // QR_code received message
  ecl::LegacyPose2D<double> QR_pose;
  double QR_pose_heading = 0;
  double last_QR_heading = QR_pose_heading;
  bool QR_new_data = false;

  ecl::LegacyPose2D<double> kalman_pose;
  double kalman_pose_heading = 0;
  double last_kalman_heading = kalman_pose_heading;
  double dx, dth;

  double linear_vel;
  double angular_vel;

  // for Robot control
  ecl::LegacyPose2D<double> obj_pose;
  char straight_axis;
  double straight_axis_value;

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
  // Go Back origin
  void go_origin(ecl::LegacyPose2D<double>);
};

void kalman_filter(bool, ecl::LegacyPose2D<double>&, double&,
                   ecl::LegacyPose2D<double>, double, double, double, double,
                   double&);

#endif
