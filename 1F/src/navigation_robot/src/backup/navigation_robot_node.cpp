
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
  std::cout << "\x1b[32m"
            << "### first PathPoint: Direction->" << first_req.getDirection()
            << "  Distance->" << first_req.getDistance() << " ###"
            << "\x1b[39m" << endl;
}

void KobukiManager::set_path(std::vector<PathPoint>& route)
{
  kobuki_req = route;
}

/**
* @brief Default constructor, needs initialisation.
*/
KobukiManager::KobukiManager()
    : dx(0.0), dth(0.0), linear_vel(linear_param), angular_vel(angular_param),
      key_file_descriptor(0),
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
bool KobukiManager::init()
{
  /*********************
  ** Parameters
  **********************/
  std::cout << "KobukiManager : using linear  vel [" << linear_vel << "]."
            << std::endl;
  std::cout << "KobukiManager : using angular vel [" << angular_vel << "]."
            << std::endl;

  /*********************
  ** PathPoint iterator
  **********************/
  init_itr();
  /*********************
  ** Kobuki
  **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = "/dev/kobuki";
  parameters.enable_acceleration_limiter = false;
  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

  return true;
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
  if (navigation_finish != true)
    RobotControll(kobuki_req, itr);
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
bool shutdown_req = false;
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
        kobuki.setBaseControl(0.0, angular_vel);
      else
        kobuki.setBaseControl(0.0, -angular_vel);
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
        kobuki.setBaseControl(0.0, angular_vel);
      else
        kobuki.setBaseControl(0.0, -angular_vel);
    }
    else
      angle_OK = true;
  }

  if (angle_OK == true)
  {
    if (diff_distance > DISTANCE_TOLERANCE)
    {
      linear_OK = false;
      kobuki.setBaseControl(linear_vel, 0);
    }
    else
      linear_OK = true;
  }

  if (angle_OK == true && linear_OK == true)
  {
    kobuki.setBaseControl(0, 0);
    // ecl::Sleep sleep(1);
    // sleep();
    reset_Dx_Dth();
    itr++;
    if (itr == kobuki_req.end())
    {
      navigation_finish = true;
      std::cout << "Navigation finished!!!" << endl;
      shutdown_req = true;
    }
    else
    {
      angle_OK = false;
      linear_OK = false;
      PathPoint next_req = *itr;
      std::cout << "\x1b[32m"
                << "### next objective: Direction->" << next_req.getDirection()
                << "  Distance->" << next_req.getDistance() << " ###"
                << "\x1b[39m" << endl;
    }
  }
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
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

  signal(SIGINT, signalHandler);
  std::cout << "\x1b[34m"
            << "--------------------Kobiki_navigator--------------------"
            << "\x1b[39m" << std::endl;
  KobukiManager kobuki_manager;
  try
  {
    // nh.getParam("input", input_method);
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

      std::vector<Route> distinations;
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
        std::cout << "\x1b[32m"
                  << "distination [" << count << "]:"
                  << "\x1b[39m" << std::endl;
        route.print_all();
        cout << endl;
      }
      bool get_correct_dist = false;
      while (!get_correct_dist)
      {
        cout << "\x1b[32m"
             << "please input distination number..."
             << "\x1b[39m" << endl;
        int dist_num_input;
        cin >> dist_num_input;
        std::vector<Route>::iterator dist_itr = distinations.begin();
        int dist_num = 0;
        for (dist_itr = distinations.begin(); dist_itr != distinations.end();
             ++dist_itr)
        {
          dist_num++;
          Route dist = *dist_itr;
          if (dist_num == dist_num_input)
          {
            kobuki_manager.set_path(dist.get_route());
            get_correct_dist = true;
          }
        }
      }
    }

    std::cout << endl
              << "\x1b[32m";
    kobuki_manager.print_req_all();
    std::cout << "\x1b[39m";

    std::cout << endl
              << "-------------now initialising...-------------" << endl;
    kobuki_manager.init();
    // std::cout << "-------------initialising complete-------------"<<endl;
    ecl::MilliSleep sleep_ms(500);
    ecl::LegacyPose2D<double> pose;

    while (!shutdown_req)
    {
      sleep_ms();
      pose = kobuki_manager.getPose();
      std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", "
                << pose.heading() << "]" << std::endl;
    }
  }
  catch (const char* errmsg)
  {
    std::cout << errmsg << endl;
    nh.setParam("input", "file");
  }
  nh.setParam("input", "file");
  return 0;
}
