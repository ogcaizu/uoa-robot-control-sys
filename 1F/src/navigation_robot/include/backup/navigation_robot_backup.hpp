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
#define EAST (-(ecl::pi)/2)
#define WEST ((ecl::pi)/2)
#define NOUTH (0)
#define SOUTH (ecl::pi)
#define ANGLE_TOLERANCE 0.1
#define DISTANCE_TOLERANCE 0.001

/*****************************************************************************
** Classes
*****************************************************************************/

/**
* @brief Keyboard remote control for our robot core (mobile base).
*
*/

class Distination
{
private:
  std::string m_direction;
  double m_distance;
public:
  Distination(string str = "None",double dis = -1){
    m_direction = str;
    m_distance = dis;
  };
  Distination(const Distination &x)
  : m_direction(x.m_direction)
  , m_distance(x.m_distance){
  };
  void print(){
    cout<<"direction: "<<m_direction<<"   distance: "<<m_distance<<endl;
  }
  double getDistance(){
    return m_distance;
  }
  std::string getDirection(){
    return m_direction;
  }
};

class DistinationList
{
private:
  std::string m_room_name;
  std::vector<Distination> m_root_list;
  std::vector<Distination>::iterator itr = m_root_list.begin();
public:
  DistinationList(string str,std::vector<Distination> list){
    m_room_name = str;
    m_root_list = list;
  };
  DistinationList(const DistinationList &x)
  : m_room_name(x.m_room_name)
  , m_root_list(x.m_root_list){
  };
  void print_all(){
    cout << "\x1b[36m" << m_room_name << "\x1b[39m" << endl;
    //cout << "\troot_list " << endl;
    print_root_list(m_root_list,itr);
  }
  void print_root_list(std::vector<Distination> &root_list, std::vector<Distination>::iterator itr){
    for(itr = root_list.begin(); itr != root_list.end(); itr++){
      Distination distination = *itr;
      distination.print();
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
  bool init();

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

private:
  // Public Member variable distruction vector
  std::vector<Distination> kobuki_req;
  std::vector<Distination>::iterator itr = kobuki_req.begin();
  bool navigation_finish = false;
  double dx, dth;
  ecl::LegacyPose2D<double> pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;

  double linear_vel;
  double angular_vel;

  // Keylogging
  int key_file_descriptor;
  struct termios original_terminal_state;

  // distruction input
  void KeyboardInput(std::vector<Distination> &);

  // distination list print
  void print_req(std::vector<Distination> &, std::vector<Distination>::iterator);

  // controll robbot
  void RobotControll(std::vector<Distination> &, std::vector<Distination>::iterator &);

};
