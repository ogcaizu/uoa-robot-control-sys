#include "../../include/navigation_robot.hpp"

#define DECIMAL_ROUND(x) (double((int)((x)*1000)) / 1000)

double global_v = 0.0;
double global_omega = 0.0;
int global_QR_heading_n = 0;
int global_kalman_heading_n = 0;

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

string KobukiManager::get_robot_mode() { return r_mode; }

void KobukiManager::set_QRpose(ecl::LegacyPose2D<double> QR)
{
  QR_pose.x(QR.x());
  QR_pose.y(QR.y());
  QR_pose.heading(QR.heading());
}

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
  if (!nh.getParam("linear_vel", linear_param))
  {
    nh.setParam("linear_vel", LINEAR_PARAM);
  }
  if (!nh.getParam("angular_vel", angular_param))
  {
    nh.setParam("angular_vel", ANGULAR_PARAM);
  }
  if (!nh.getParam("input", input_method))
  {
    nh.setParam("input", "file");
  }
  linear_vel = linear_param;
  angular_vel = angular_param;
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
  robot_state_pub = nh.advertise<office_guide_robot::r_state>("state", 100);

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
  std::string iso_time_str = boost::posix_time::to_simple_string(my_posix_time);
  pub_msg.time = iso_time_str;
  pub_msg.r_mode = r_mode;
  pub_msg.pos.x = kalman_pose.x();
  pub_msg.pos.y = kalman_pose.y();
  pub_msg.pos.theta = kalman_pose_heading;
  robot_state_pub.publish(pub_msg);
}

/*****************************************************************************
** Subscriber CallBack
*****************************************************************************/
void KobukiManager::msgCallback(const office_guide_robot::r_req::ConstPtr& msg)
{
  cout << "***receive msg***" << endl;
  cout << msg->time << endl;
  cout << msg->r_cmd << endl;
  cout << msg->pos.x << "," << msg->pos.y << endl;
  if (msg->r_cmd == "Navi")
  {
    std::stringstream ss;
    ss << msg->pos.x << "," << msg->pos.y;
    ss >> receive_msg_goal;
  }
  msg_receive = true;
}

void KobukiManager::msgCallback1(
    const office_guide_robot::Position::ConstPtr& msg)
{
  // ROS_INFO("x = %f \t y = %f \t theta = %f", msg->x, msg->y, msg->theta);
  QR_new_data = true;
  QR_pose.x(msg->x);
  QR_pose.y(msg->y);
  QR_pose.heading(msg->theta);
  QR_pose_heading = msg->theta;
  if ((last_QR_heading - QR_pose_heading) > ecl::pi * 1.5)
    global_QR_heading_n++;
  else if ((last_QR_heading - QR_pose_heading) < (-ecl::pi * 1.5))
    global_QR_heading_n--;
  last_QR_heading = QR_pose_heading;
  QR_pose_heading = QR_pose_heading + 2.0 * global_QR_heading_n * ecl::pi;
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

/*****************************************************************************
** Implementation [Go back the origin]
*****************************************************************************/ // new_origin
void KobukiManager::go_origin(ecl::LegacyPose2D<double> current_pose)
{
  ecl::LegacyPose2D<double> origin(0.0, 0.0, 0.0);
  kobuki::DiffDrive diff_drive;
  ecl::linear_algebra::Vector2d route;
  ecl::DifferentialDrive::Kinematics kinematics = diff_drive.kinematics();
  route = kinematics.PartialInverse(current_pose,
                                    origin); // route(0):ds, route(1):dtheta
  double targetDir = 0.0;
  // 現在位置から原点への方位（ラジアン）を取得
  targetDir = atan2((double)(origin.y() - current_pose.y()),
                    (double)(origin.x() - current_pose.x()));
  cout << "Dist = " << route(0) << ", Dir = " << targetDir << endl;

  // 指定方位まで旋回する
  if (turn_origin_ok != true)
  {
    if (current_pose.heading() < targetDir)
    {
      kobuki.setBaseControl(0, ANGULAR_PARAM);
    }
    else
    {
      kobuki.setBaseControl(0, -ANGULAR_PARAM);
    }
    if ((current_pose.heading() > (targetDir - ANGLE_TOLERANCE)) &&
        (current_pose.heading() < (targetDir + ANGLE_TOLERANCE)))
    {
      turn_origin_ok = true;
    }
  }

  // 指定距離まで前進する
  else if (forward_origin_ok != true)
  {
    if (route(0) > DISTANCE_TOLERANCE)
    {
      kobuki.setBaseControl(LINEAR_PARAM, 0);
    }
    else
    {
      forward_origin_ok = true;
    }
  }

  else if (turn_origin_ok == true && forward_origin_ok == true)
  {
    origin_ok == true;
  }
}

/*****************************************************************************
** Implementation [Process stream data]
*****************************************************************************/
void KobukiManager::processStreamData()
{
  prev_time = (double)time.tv_sec + (double)time.tv_nsec * 1.0E-9;
  clock_gettime(CLOCK_REALTIME, &time);
  delta_t = (((double)time.tv_sec + (double)time.tv_nsec * 1.0E-9) - prev_time);
  delta_t = (double)((int)(delta_t * 1.0E9)) * 1.0E-9;

  ros::spinOnce();
  ecl::LegacyPose2D<double> odom_pose_update;
  ecl::linear_algebra::Vector3d odom_pose_update_rates;
  kobuki.updateOdometry(odom_pose_update, odom_pose_update_rates);
  odom_pose *= odom_pose_update;
  dx += odom_pose_update.x();
  dth += odom_pose_update.heading();
  // cout << "Odom_update_rates: x = " << odom_pose_update_rates(0) << "y =
  // "<<odom_pose_update_rates(1) << "theta = " << odom_pose_update_rates(2)
  // <<endl;
  global_v = odom_pose_update_rates(0);
  global_omega = odom_pose_update_rates(2);
  // std::cout << dx << ", " << dth << std::endl;
  // std::cout << kobuki.getHeading() << ", " << odom_pose.heading() <<
  // std::endl;
  // std::cout << "[" << odom_pose.x() << ", " << odom_pose.y() << ", " <<
  // odom_pose.heading()
  // << "]" << std::endl;
  PublishRobotState();

  if (get_robot_mode() == "Navi")
  {
    // this is log print out ->
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    boost::posix_time::hours time_difference(9);
    my_posix_time += time_difference;
    std::string iso_time_str =
        boost::posix_time::to_simple_string(my_posix_time);
    // Print Robot State !! commented at 10/12
    // cout << iso_time_str << ", " << odom_pose.x() << ", " << odom_pose.y()
    //     << ", " << odom_pose.heading() << ", " << global_v << ", "
    //     << global_omega << endl;
    // this is log print out end <-
    bool QR_get = false;
    if (QR_new_data == true)
    {
      QR_get = true;
      QR_new_data = false;
    }
    kalman_filter(QR_get, kalman_pose, kalman_pose_heading, QR_pose,
                  QR_pose_heading, global_v, global_omega, delta_t, kalman_dx);

//    if ((last_kalman_heading - 2.0*global_kalman_heading_n * ecl::pi) - (kalman_pose_heading - 2.0*global_kalman_heading_n * ecl::pi) > ecl::pi * 1.5)
//      global_kalman_heading_n++;
//    else if ((last_kalman_heading -2.0 * global_kalman_heading_n * ecl::pi) - (kalman_pose_heading-2.0 * global_kalman_heading_n * ecl::pi) < (-ecl::pi * 1.5))
//      global_kalman_heading_n--;
//    last_kalman_heading = kalman_pose_heading;
    // kalman_pose_heading = kalman_pose_heading - 2.0 * global_kalman_heading_n
    // * ecl::pi;

    cout << iso_time_str << ", " << setw(12) << left << kalman_pose.x() << ", "
         << setw(12) << left << kalman_pose.y() <<", " << setw(12) << left<< kalman_pose_heading << ", " << setw(13) << left << global_v << ", "
         << setw(13) << left << global_omega << ", " << QR_get;
    if (QR_get == true)
      cout << ", " << setw(13) << left << QR_pose.x() << ", " << setw(13)
           << left << QR_pose.y() << ", " << setw(13) << left << QR_pose_heading
           << ", n: " << global_QR_heading_n << endl;
    else
      cout << endl;
  }

  if (navigation_finish != true)
  {
    set_robot_mode_navi();
    RobotControll(kobuki_req, itr);
  }
  if (shutdown_req)
  {
    kobuki.setBaseControl(0, 0);
    kobuki.disable();
  }
}

ecl::LegacyPose2D<double> KobukiManager::get_odomPose() { return odom_pose; }

ecl::LegacyPose2D<double> KobukiManager::get_kalmanPose()
{
  return kalman_pose;
}

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
** Robot Controll
*****************************************************************************/
void KobukiManager::RobotControll(std::vector<PathPoint>& kobuki_req,
                                  std::vector<PathPoint>::iterator& itr)
{
  PathPoint path_point = *itr;
  std::string direction = path_point.getDirection();
  double distance = path_point.getDistance();
  double obj_angle = 0.0;
  ecl::LegacyPose2D<double> position =
      get_kalmanPose(); // we will change get_kalmanPose();
  static bool angle_OK = false;
  static bool linear_OK = false;
  static bool turn_OK = false;
  char c = direction[0];
  float tolerance = 0;
  if (turn_OK)
    tolerance = 0;
  else
    tolerance = ANGLE_TOLERANCE;

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

  double head = kalman_pose_heading - 2.0 * global_QR_heading_n * ecl::pi;
  ;
  double diff_angle = obj_angle - head;
  // double diff_distance = distance - kalman_dx; // getDx(); for kalman test
  double diff_distance;
  if (straight_axis == 'x')
  {
    if (obj_angle == NOUTH)
      diff_distance = obj_pose.x() - position.x();
    else if (obj_angle == SOUTH)
      diff_distance = position.x() - obj_pose.x();
  }
  else if (straight_axis == 'y')
  {
    if (obj_angle == EAST)
      diff_distance = position.y() - obj_pose.y();
    else if (obj_angle == WEST)
      diff_distance = obj_pose.y() - position.y();
  }
  // cout<<"straight_axis = "<<straight_axis<<", diff_distance =
  // "<<diff_distance<<endl;

  // while diff_anble is larger than tollelance, robot roll on the spot
  double straight_vel;
  if (turn_OK == true)
  {
    straight_vel = linear_vel;
    if (obj_angle == SOUTH)
    {
      if (head > 0) {
        if ((position.y() - obj_pose.y()) > 0)
          angular_vel = ANGLE_CONST * std::abs(diff_angle) +
                        DIFF_STRAIGHT * std::sqrt(position.y() - obj_pose.y());
        else
          angular_vel =
              ANGLE_CONST * std::abs(diff_angle) -
              DIFF_STRAIGHT * std::sqrt(std::abs(position.y() - obj_pose.y()));
      }
      else {
        if ((position.y() - obj_pose.y()) > 0)
          angular_vel = ANGLE_CONST * (-((ecl::pi) + head)) +
                         DIFF_STRAIGHT * std::sqrt(position.y() - obj_pose.y());
        else
          angular_vel =
              ANGLE_CONST * (-((ecl::pi) + head)) -
              DIFF_STRAIGHT * std::sqrt(std::abs(position.y() - obj_pose.y()));
      }
    }
    if (obj_angle == EAST)
    {
      if (-(ecl::pi / 2) < head && head < (ecl::pi / 2))
      {
        if ((obj_pose.x() - position.x()) > 0)
          angular_vel = ANGLE_CONST * (-std::abs(diff_angle)) +
                        DIFF_STRAIGHT * std::sqrt(obj_pose.x() - position.x());
        else
          angular_vel =
              ANGLE_CONST * (-std::abs(diff_angle)) -
              DIFF_STRAIGHT * std::sqrt(std::abs(obj_pose.x() - position.x()));
      }
      else
      {
        if ((obj_pose.x() - position.x()) > 0)
          angular_vel = ANGLE_CONST * std::abs(diff_angle) +
                        DIFF_STRAIGHT * std::sqrt(obj_pose.x() - position.x());
        else
          angular_vel =
              ANGLE_CONST * std::abs(diff_angle) -
              DIFF_STRAIGHT * std::sqrt(std::abs(obj_pose.x() - position.x()));
      }
    }
    if (obj_angle == WEST)
    {
      if (-(ecl::pi / 2) < head && head < (ecl::pi / 2))
      {
        if ((position.x() - obj_pose.x()) > 0)
          angular_vel = ANGLE_CONST * std::abs(diff_angle) +
                        DIFF_STRAIGHT * std::sqrt(position.x() - obj_pose.x());
        else
          angular_vel =
              ANGLE_CONST * std::abs(diff_angle) -
              DIFF_STRAIGHT * std::sqrt(std::abs(position.x() - obj_pose.x()));
      }
      else
      {
        if ((position.x() - obj_pose.x()) > 0)
          angular_vel = ANGLE_CONST * (-std::abs(diff_angle)) +
                        DIFF_STRAIGHT * std::sqrt(position.x() - obj_pose.x());
        else
          angular_vel =
              ANGLE_CONST * (-std::abs(diff_angle)) -
              DIFF_STRAIGHT * std::sqrt(std::abs(position.x() - obj_pose.x()));
      }
    }
    if (obj_angle == NOUTH)
    {
      if (head < 0)
      {
        if ((obj_pose.y() - position.y()) > 0)
          angular_vel = ANGLE_CONST * std::abs(diff_angle) +
                        DIFF_STRAIGHT * std::sqrt(obj_pose.y() - position.y());
        else
          angular_vel =
              ANGLE_CONST * std::abs(diff_angle) -
              DIFF_STRAIGHT * std::sqrt(std::abs(obj_pose.y() - position.y()));
      }
      else
      {
        if ((obj_pose.y() - position.y()) > 0)
          angular_vel = ANGLE_CONST * (-std::abs(diff_angle)) +
                        DIFF_STRAIGHT * std::sqrt(obj_pose.y() - position.y());
        else
          angular_vel =
              ANGLE_CONST * (-std::abs(diff_angle)) -
              DIFF_STRAIGHT * std::sqrt(std::abs(obj_pose.y() - position.y()));
      }
    }
  }
  else
  {
    straight_vel = 0.0;
    if (obj_angle == SOUTH)
    {
      if (head > 0)
        angular_vel = angular_param;
      else
        angular_vel = -angular_param;
    }
    if (obj_angle == EAST)
    {
      if (-(ecl::pi / 2) < head && head < (ecl::pi / 2))
        angular_vel = -angular_param;
      else
        angular_vel = angular_param;
    }
    if (obj_angle == WEST)
    {
      if (-(ecl::pi / 2) < head && head < (ecl::pi / 2))
        angular_vel = angular_param;
      else
        angular_vel = -angular_param;
    }
    if (obj_angle == NOUTH)
    {
      if (head < 0)
        angular_vel = angular_param;
      else
        angular_vel = -angular_param;
    }
  }

  if (obj_angle == SOUTH)
  {
    if (((ecl::pi - head > tolerance / 2) && (head >= 0)) ||
        ((ecl::pi + head > tolerance / 2) && (head < 0)))
    {
      angle_OK = false;
      kobuki.setBaseControl(straight_vel, angular_vel);
    }
    else
    {
      angle_OK = true;
      turn_OK = true;
    }
  }
  else
  {
    if (std::abs(diff_angle) > tolerance)
    {
      angle_OK = false;
      kobuki.setBaseControl(straight_vel, angular_vel);
    }
    else
    {
      angle_OK = true;
      turn_OK = true;
    }
  }
  if (angle_OK == true)
    kobuki.setBaseControl(straight_vel, 0);

  if (diff_distance > DISTANCE_TOLERANCE)
    linear_OK = false;
  else
    linear_OK = true;

  if (linear_OK == true && turn_OK == true)
  {
    kobuki.setBaseControl(0, 0);
    linear_OK = false;
    angle_OK = false;
    turn_OK = false;
    reset_Dx_Dth();
    kalman_dx = 0; // for kalman test

    itr++;
    if (itr == kobuki_req.end())
    {
      navigation_finish = true;
      set_robot_mode_standby();
      std::cout << endl
                << "\e[91m"
                << "Navigation finished!!!"
                << "\e[39m" << endl;
    }
    else
    {
      turn_OK = false;
      linear_OK = false;
      angle_OK = false;
      PathPoint next_req = *itr;
      std::cout << "\x1b[95m"
                << "### next objective: Direction->" << next_req.getDirection()
                << "  Distance->" << next_req.getDistance() << " ###"
                << "\x1b[39m" << endl;

      string dir = next_req.getDirection();
      if (dir == "Nouth" || dir == "South")
      {
        straight_axis = 'x';
        if (dir == "Nouth")
          obj_pose.x(DECIMAL_ROUND(obj_pose.x() + next_req.getDistance()));
        else if (dir == "South")
          obj_pose.x(DECIMAL_ROUND(obj_pose.x() - next_req.getDistance()));
        // straight_axis_value =
      }
      else if (dir == "East" || dir == "West")
      {
        straight_axis = 'y';
        if (dir == "West")
          obj_pose.y(DECIMAL_ROUND(obj_pose.y() + next_req.getDistance()));
        else if (dir == "East")
          obj_pose.y(DECIMAL_ROUND(obj_pose.y() - next_req.getDistance()));
      }
      cout << "Next goal : " << obj_pose.x() << ", " << obj_pose.y() << endl;
    }
  }
}

/*****************************************************************************
** Get/Set Distinaion
*****************************************************************************/
void KobukiManager::get_distination(KobukiManager& kobuki_manager)
{
  if (input_method == "keyboard")
    kobuki_manager.req_input_keyboard();
  else
  {
    const char* fileName = FILENAME;
    std::ifstream ifs;
    ifs.open(fileName);
    std::string str;
    if (!ifs)
    {
      std::cerr << "error: Can not open \"dist.txt\" filename: " << fileName
                << std::endl;
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
      while (std::getline(ss, data, ' ')) //スペースでパース
      {
        v.push_back(data); //パースしたデータをvectorにpush_back
      }
      std::vector<std::string>::iterator itr = v.begin();
      std::string direction;
      double distance;
      std::string goal_point = *itr;
      itr++;
      std::string dist_room = *itr;
      itr++;
      for (itr; itr != v.end(); ++itr)
      {
        direction = *itr;
        itr++;
        distance = std::stod(*itr);
        path_list.push_back(PathPoint(direction, distance));
      }
      Route route(goal_point, dist_room, path_list);
      // cout << route.get_goal_point() << endl;
      distinations.push_back(route);
    }
    std::vector<PathPoint> home_path{PathPoint("Nouth", 0)};
    Route home_route_dummy("0,0", "home_dummy", home_path);
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
    sleep(1);
    // for manual select distination
    int dist_num_input;
    if (!msg_receive)
    {
      cout << "\x1b[32m"
           << "please input distination number..."
           << "\x1b[39m" << endl;
      for (cin >> dist_num_input; !cin; cin >> dist_num_input)
      {
        cin.clear();
        cin.ignore();
        cout << "please input correct number!!" << endl;
      }
    }

    for (dist_num = 0, dist_itr = distinations.begin();
         dist_itr != distinations.end(); ++dist_itr)
    {
      dist_num++;
      Route dist = *dist_itr;
      /* for subscribe message */
      if (msg_receive == true)
      {
        if (dist.get_goal_point() == receive_msg_goal)
        {
          kobuki_manager.set_path(dist.get_route());
          kobuki_manager.set_return_path(dist.get_return_route());
          Route route("0,0", "home", kobuki_req_home);
          distinations.pop_back();
          distinations.push_back(route);

          get_correct_dist = true;
        }
      }
      /* for manual select distination */
      else
      {
        if (dist_num == dist_num_input)
        {
          kobuki_manager.set_path(dist.get_route());
          kobuki_manager.set_return_path(dist.get_return_route());
          Route route("0,0", "home", kobuki_req_home);
          distinations.pop_back();
          distinations.push_back(route);

          get_correct_dist = true;
        }
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
  clock_gettime(CLOCK_REALTIME, &time);
  /*********************
  ** PathPoint iterator initialise
  **********************/
  init_itr();
  PathPoint pp = *itr;
  string dir = pp.getDirection();
  if (dir == "Nouth" || dir == "South")
  {
    straight_axis = 'x';
    if (dir == "Nouth")
      obj_pose.x(DECIMAL_ROUND(obj_pose.x() + pp.getDistance()));
    else if (dir == "South")
      obj_pose.x(DECIMAL_ROUND(obj_pose.x() - pp.getDistance()));
  }
  else if (dir == "East" || dir == "West")
  {
    straight_axis = 'y';
    if (dir == "West")
      obj_pose.y(DECIMAL_ROUND(obj_pose.y() + pp.getDistance()));
    else if (dir == "East")
      obj_pose.y(DECIMAL_ROUND(obj_pose.y() - pp.getDistance()));
  }
  cout << "Next goal : " << obj_pose.x() << ", " << obj_pose.y() << endl;
  /******************************************************
  ** kobuki connect in older to callback function
  *******************************************************/
  slot_stream_data.connect("/kobuki/stream_data");
}

// void KobukiManager::kobuki_stop(){kobuki.setBaseControl(0, 0);}

ecl::LegacyPose2D<double> KobukiManager::get_QRpose() { return QR_pose; }
