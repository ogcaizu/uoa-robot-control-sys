
#include "../include/navigation_robot.hpp"

/*****************************************************************************
** Implementation
*****************************************************************************/
void KobukiManager::req_input_keyboard()
{
  KobukiManager::KeyboardInput(kobuki_req);
}

void KobukiManager::print_req_all()
{
  KobukiManager::print_req(kobuki_req, itr);
}

void KobukiManager::reset_Dx_Dth()
{
  dx = 0;
  dth = 0;
}

void KobukiManager::reset_Dx() { dx = 0; }

double KobukiManager::getDx() { return dx; }

void KobukiManager::init_itr()
{
  itr = kobuki_req.begin();
  PathPoint first_req = *itr;
  std::cout << "\x1b[95m"
            << "### first PathPoint: Direction->" << first_req.getDirection()
            << "  Distance->" << first_req.getDistance() << " ###"
            << "\x1b[39m" << endl;
}

void KobukiManager::set_path(std::vector<PathPoint>& route)
{
  kobuki_req = route;
}

void KobukiManager::set_return_path(std::vector<PathPoint>& route)
{
  kobuki_req_home = route;
}

void KobukiManager::set_robot_mode_standby() { r_mode = "Standby"; }

void KobukiManager::set_robot_mode_navi() { r_mode = "Navi"; }

/**
* @brief Default constructor, needs initialisation.
*/
KobukiManager::KobukiManager()
    : dx(0.0), dth(0.0), linear_vel(linear_param), angular_vel(angular_param),
      r_mode("Standby"), key_file_descriptor(0),
      slot_stream_data(&KobukiManager::processStreamData, *this)
{
  tcgetattr(key_file_descriptor,
            &original_terminal_state); // get terminal properties
}

KobukiManager::~KobukiManager()
{
  kobuki.setBaseControl(
      0, 0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
* @brief Initialises the node.
*/
bool KobukiManager::init(ros::NodeHandle& nh)
{
  std::cout << endl
            << "-------------now initialising...-------------" << endl;
  /*********************
  ** Parameters
  **********************/
  std::cout << "KobukiManager : using linear  vel [" << linear_vel << "]."
            << std::endl;
  std::cout << "KobukiManager : using angular vel [" << angular_vel << "]."
            << std::endl;
  /*********************
  ** Kobuki
  **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = "/dev/kobuki";
  parameters.enable_acceleration_limiter = false;
  kobuki.init(parameters);
  kobuki.enable();

  /*********************
  ** publisher initialize
  **********************/
  robot_state_pub = nh.advertise<service_robot_msg::RobotState>("state", 100);

  std::cout << "-------------initialising complete-------------" << endl;

  return true;
}

/*****************************************************************************
** Implementation [Publisher]
*****************************************************************************/
void KobukiManager::PublishRobotState()
{
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  boost::posix_time::hours time_difference(9);
  my_posix_time += time_difference;
  std::string iso_time_str =
      boost::posix_time::to_iso_extended_string(my_posix_time);
  pub_msg.time = iso_time_str;
  pub_msg.r_mode = r_mode;
  pub_msg.pos.x = pose.x();
  pub_msg.pos.y = pose.y();
  pub_msg.pos.theta = pose.heading();
  robot_state_pub.publish(pub_msg);
}

/*****************************************************************************
** Implementation [Keyboard]
*****************************************************************************/
void KobukiManager::KeyboardInput(std::vector<PathPoint>& kobuki_req)
{
  char c;
  string direction;
  double distance;
  std::cout << "\x1b[31m"
            << "Please input arrow_key and distance[m]"
            << "\x1b[39m" << endl;
  std::cout << "\x1b[31m"
            << "[q] to finish input "
            << "\x1b[39m" << endl;
  while (1)
  {
    //キーボードからの入力をすぐさま反映する
    system("stty -echo -icanon min 1 time 0");
    c = '\n';
    direction = '\n';
    distance = -1;
    try
    {
      std::cin >> c;
      if (c == 'q')
      {
        system("stty echo -icanon min 1 time 0");
        if (kobuki_req.empty())
          throw("input is empty");
        else
          break;
      }
      else if (c == '\e')
      {
        std::cin >> c;
        if (c == '[')
        {
          std::cin >> c;
        }
        else
        {
          // cout<<"Please input arrow_key !!"<<endl;
          throw("Please input arrow_key !!");
          // exit(1);
        }
      }
      else
      {
        throw("Please input arrow_key !!");
        // cout<<"Please input arrow_key !"<<endl;
        // exit(2);
      }

      switch (c)
      {
      case 'D':
      {
        direction = "West";
        std::cout << "direction: West";
        break;
      }
      case 'C':
      {
        direction = "East";
        std::cout << "direction: East";
        break;
      }
      case 'A':
      {
        direction = "Nouth";
        std::cout << "direction: Nouth";
        break;
      }
      case 'B':
      {
        direction = "South";
        std::cout << "direction: South";
        break;
      }
      default:
      {
        throw("Please input arrow_key !!");
        // cout<<"Please input arrow_key !!!";
        // exit(3);
        break;
      }
      }

      std::cout << "   distance: ";
      system("stty echo -icanon min 1 time 0");
      if (std::cin >> distance)
      {
        // cout<< " distance: "<<distance<<endl;
        kobuki_req.push_back(PathPoint(direction, distance));
        // return PathPoint(direction,distance);
      }
      else
      {
        throw("Please input correct distance[m]");
        // cout<<endl<<"Please input correct distance[m]!"<<endl;
        // exit(4);
      }
    }
    catch (const char* errmsg)
    {
      system("stty echo -icanon min 1 time 0");
      std::cout << errmsg << endl;
      std::cout << endl
                << "Please input correct Format!!" << endl;
      throw("error");
      // nh.setParam("input","file");
      // exit(-1);
    }
  }
}

void KobukiManager::processStreamData()
{
  ecl::LegacyPose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  pose *= pose_update;
  dx += pose_update.x();
  dth += pose_update.heading();
  // std::cout << dx << ", " << dth << std::endl;
  // std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
  // std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading()
  // << "]" << std::endl;
  PublishRobotState();
  if (navigation_finish != true)
  {
    set_robot_mode_navi();
    RobotControll(kobuki_req, itr);
  }
  if (shutdown_req)
  {
    cout << "shutdown called!!!" << endl;
    kobuki.setBaseControl(0, 0);
    kobuki.disable();
  }
}

ecl::LegacyPose2D<double> KobukiManager::getPose() { return pose; }

/*****************************************************************************
** Print request list
*****************************************************************************/
void KobukiManager::print_req(std::vector<PathPoint>& kobuki_req,
                              std::vector<PathPoint>::iterator itr)
{
  for (itr = kobuki_req.begin(); itr != kobuki_req.end(); ++itr)
  {
    PathPoint PathPoint = *itr;
    PathPoint.print();
  }
}
/*****************************************************************************
** Signal Handler
*****************************************************************************/
void signalHandler(int signum) { shutdown_req = true; }

/*****************************************************************************
** Robot Controll
*****************************************************************************/
void KobukiManager::RobotControll(std::vector<PathPoint>& kobuki_req,
                                  std::vector<PathPoint>::iterator& itr)
{
  PathPoint path_point = *itr;
  std::string direction = path_point.getDirection();
  double distance = path_point.getDistance();
  double obj_angle = 0.0;
  ecl::LegacyPose2D<double> position = getPose();
  static bool angle_OK = false;
  static bool linear_OK = false;
  static bool turn_OK = false;
  char c = direction[0];
  switch (c)
  {
  case 'E':
  {
    obj_angle = EAST;
    break;
  }
  case 'W':
  {
    obj_angle = WEST;
    break;
  }
  case 'S':
  {
    obj_angle = SOUTH;
    break;
  }
  case 'N':
  {
    obj_angle = NOUTH;
    break;
  }
  }
  // while diff_anble is larger than tollelance, robot roll on the spot
  double straight_vel;
  if (turn_OK = true)
    straight_vel = linear_vel;
  else
    straight_vel = 0.0;

  double head = position.heading();
  double diff_angle = obj_angle - head;
  double diff_distance = distance - getDx();
  if (obj_angle == SOUTH)
  {
    if ((ecl::pi - head > ANGLE_TOLERANCE / 2) &&
        (ecl::pi + head > ANGLE_TOLERANCE / 2))
    {
      angle_OK = false;
      if (ecl::pi - head < ecl::pi + head)
        kobuki.setBaseControl(straight_vel, angular_vel);
      else
        kobuki.setBaseControl(straight_vel, -angular_vel);
    }
    else
      angle_OK = true;
  }
  else if (obj_angle != SOUTH)
  {
    if (std::abs(diff_angle) > ANGLE_TOLERANCE)
    {
      // reset_Dx();
      angle_OK = false;
      if (diff_angle > 0)
        kobuki.setBaseControl(straight_vel, angular_vel);
      else
        kobuki.setBaseControl(straight_vel, -angular_vel);
    }
    else
      angle_OK = true;
  }

  if (angle_OK == true)
  {
    if (diff_distance > DISTANCE_TOLERANCE)
    {
      linear_OK = false;
      kobuki.setBaseControl(straight_vel, 0);
    }
    else
      linear_OK = true;
  }

  if (angle_OK == true && linear_OK == true)
  {
    kobuki.setBaseControl(0, 0);
    linear_OK = false;
    angle_OK = false;
    turn_OK = false;
    // ecl::Sleep sleep(1);
    // sleep();
    reset_Dx_Dth();
    itr++;
    if (itr == kobuki_req.end())
    {
      navigation_finish = true;
      set_robot_mode_standby();
      std::cout << endl
                << "\e[91m"
                << "Navigation finished!!!"
                << "\e[39m" << endl;
      // shutdown_req = true;
    }
    else
    {
      PathPoint next_req = *itr;
      std::cout << "\x1b[95m"
                << "### next objective: Direction->" << next_req.getDirection()
                << "  Distance->" << next_req.getDistance() << " ###"
                << "\x1b[39m" << endl;
    }
  }
}

/*****************************************************************************
** Get/Set Distinaion
*****************************************************************************/
void KobukiManager::get_distination(std::string input_method,
                                    KobukiManager& kobuki_manager)
{
  if (input_method == "keyboard")
    kobuki_manager.req_input_keyboard();
  else
  {
    const char* fileName =
        "/home/kota/catkin_ws/src/navigation_robot/src/dist.txt";
    std::ifstream ifs;
    ifs.open(fileName);
    std::string str;
    if (!ifs)
    {
      std::cerr << "error: Can not open \"dist.txt\"" << std::endl;
      throw("error");
    }

    std::string row;  //道順リストを一行ずつ格納する変数
    std::string data; //パース後のデータを格納する変数
    int count = 0;    //目的地数
    while (getline(ifs, row))
    { //一行ずつ読み込み
      if (row == "")
        break; //最終行が改行のみの場合パースでセグフォするのでブレイク
      count++;
      std::vector<PathPoint> path_list;
      std::vector<std::string> v; //パースしたデータを格納するstring型のvector
      std::stringstream ss(row); //パース処理のためstreamにする
      while (std::getline(ss, data, ' '))
      {                    //スペースでパース
        v.push_back(data); //パースしたデータをvectorにpush_back
      }
      std::vector<std::string>::iterator itr = v.begin();
      std::string direction;
      double distance;
      std::string dist_room = *itr;
      itr++;
      for (itr; itr != v.end(); ++itr)
      {
        direction = *itr;
        itr++;
        distance = std::stod(*itr);
        path_list.push_back(PathPoint(direction, distance));
      }
      Route route(dist_room, path_list);
      distinations.push_back(route);
    }
    std::vector<PathPoint> home_path{PathPoint("Nouth", 0)};
    Route home_route_dummy("home_dummy", home_path);
    distinations.push_back(home_route_dummy);
  }
}

void KobukiManager::set_distination(KobukiManager& kobuki_manager)
{
  int dist_num = 0;
  for (dist_num = 0, dist_itr = distinations.begin();
       dist_itr != distinations.end(); ++dist_itr)
  {
    dist_num++;
    Route dist = *dist_itr;
    std::cout << endl
              << "\x1b[32m"
              << "distination [" << dist_num << "]:"
              << "\x1b[39m" << std::endl;
    dist.print_all();
  }

  bool get_correct_dist = false;
  while (!get_correct_dist)
  {
    cout << "\x1b[32m"
         << "please input distination number..."
         << "\x1b[39m" << endl;
    int dist_num_input;
    for (cin >> dist_num_input; !cin; cin >> dist_num_input)
    {
      cin.clear();
      cin.ignore();
      cout << "please input correct number!!" << endl;
    }

    for (dist_num = 0, dist_itr = distinations.begin();
         dist_itr != distinations.end(); ++dist_itr)
    {
      dist_num++;
      Route dist = *dist_itr;
      if (dist_num == dist_num_input)
      {
        kobuki_manager.set_path(dist.get_route());
        kobuki_manager.set_return_path(dist.get_return_route());
        Route route("home", kobuki_req_home);
        distinations.pop_back();
        distinations.push_back(route);

        get_correct_dist = true;
      }
    }
  }
  std::cout << endl
            << "\x1b[92m"
            << "================Route================" << endl;
  kobuki_manager.print_req_all();
  std::cout << "====================================="
            << "\x1b[39m" << endl;
}

void KobukiManager::kobuki_start()
{
  /*********************
  ** PathPoint iterator initialise
  **********************/
  init_itr();
  /******************************************************
  ** kobuki connect in older to callback function
  *******************************************************/
  slot_stream_data.connect("/kobuki/stream_data");
}

// void KobukiManager::kobuki_stop(){kobuki.setBaseControl(0, 0);}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);
  ros::init(argc, argv, "navigation_robot_node");
  ros::NodeHandle nh("~");

  if (!nh.getParam("linear_vel", linear_param))
  {
    nh.setParam("linear_vel", LINEAR_PARAM);
  }
  if (!nh.getParam("angular_vel", angular_param))
  {
    nh.setParam("angular_vel", ANGULAR_PARAM);
  }
  std::string input_method = "";
  if (!nh.getParam("input", input_method))
  {
    nh.setParam("input", "file");
  }

  std::cout << "\x1b[34m"
            << "--------------------Kobiki_navigator--------------------"
            << "\x1b[39m" << std::endl;
  KobukiManager kobuki_manager;
  try
  {

    kobuki_manager.get_distination(input_method, kobuki_manager);
    kobuki_manager.init(nh);

    while (!shutdown_req)
    {
      kobuki_manager.set_distination(kobuki_manager);
      kobuki_manager.navigation_finish = false;
      kobuki_manager.kobuki_start();

      ecl::MilliSleep sleep_ms(200);
      ecl::LegacyPose2D<double> pose;
      while (!kobuki_manager.navigation_finish)
      {
        sleep_ms();
        pose = kobuki_manager.getPose();
        std::cout.setf(std::ios::left, std::ios::adjustfield);
        std::cout << "current pose: [" << std::setw(13) << pose.x() << ", "
                  << std::setw(13) << pose.y() << ", " << std::setw(13)
                  << pose.heading() << "]" << endl;

        std::cout.setf(std::ios::right, std::ios::adjustfield);
      }
    }
  }
  catch (const char* errmsg)
  {
    sleep(1);
    std::cout << errmsg << endl;
    nh.setParam("input", "file");
  }
  sleep(2);
  nh.setParam("input", "file");
  return 0;
}
