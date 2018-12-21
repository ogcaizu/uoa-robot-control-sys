#include "../include/navigation_robot.hpp"

bool shutdown_req = false;
bool msg_receive = false;
std::string receive_msg_goal = "None";

/*****************************************************************************
** Implementation
*****************************************************************************/
/*****************************************************************************
** Signal Handler
*****************************************************************************/
void signalHandler(int signum)
{
  cout << "SIGINT called!! Shutdown..." << endl;
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);
  ros::init(argc, argv, "navigation_robot_node");
  ros::NodeHandle nh("~");

  std::cout << "\x1b[34m"
            << "--------------------Kobiki_navigator--------------------"
            << "\x1b[39m" << std::endl;
  KobukiManager kobuki_manager;
  // kobuki_manager.init(nh);
  kobuki_manager.init(nh);
  // subscriber declaration
  ros::Subscriber robot_request_sub;
  robot_request_sub = nh.subscribe("request", 100, &KobukiManager::msgCallback,
                                   &kobuki_manager); // subscriber initialize
  ros::Subscriber QR_position_sub;
  QR_position_sub = nh.subscribe("Position", 100, &KobukiManager::msgCallback1,
                                 &kobuki_manager);

  try
  {

    kobuki_manager.get_distination(kobuki_manager);
    // kobuki_manager.init(nh); move to line 565
    ros::Rate r(10);
    while (!shutdown_req)
    {
      msg_receive = false;
      cout << "waiting to receive a message..."
           << endl; // <<"if you want to skip, press 'q'."<<endl;
      while (!msg_receive)
      {
        // cout <<"roop"<<endl;
        // char c='a';
        // std::future<void> f = std::async([](char& _c){cout<<"async"<<endl;
        // std::cin >> _c; },std::ref(c));
        // std::future_status result =f.wait_for(std::chrono::seconds(1));
        //	if(result == std::future_status::timeout){
        // cout<<"time out"<<endl;
        // continue;
        //}
        //        else{
        // cout << "in"<<endl;
        //	  if(c=='q') break;
        //	}

        ros::spinOnce();
        kobuki_manager.PublishRobotState();
        r.sleep();
      }

      kobuki_manager.set_distination(kobuki_manager);
      kobuki_manager.navigation_finish = false;
      kobuki_manager.kobuki_start();

      ecl::MilliSleep sleep_ms(100);
      ecl::LegacyPose2D<double> odom_pose;
      while (!kobuki_manager.navigation_finish)
      {
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        boost::posix_time::hours time_difference(9);
        my_posix_time += time_difference;
        std::string iso_time_str =
            boost::posix_time::to_simple_string(my_posix_time);
        // sleep_ms();
        odom_pose = kobuki_manager.get_odomPose();
        // Print Robot State !! commented at 10/12
        // cout << iso_time_str << ", " << odom_pose.x() << ", " <<
        // odom_pose.y() << ",
        // "<< odom_pose.heading() << endl;

        if (!ros::ok())
        {
          cout << "SIGINT called!! Shutdown..." << endl;
          shutdown_req = true;
          break;
        }
        //        std::cout.setf(std::ios::left, std::ios::adjustfield);
        //        std::cout << "current pose: [" << std::setw(13) <<
        //        odom_pose.x() <<
        //        ", "
        //
        // << std::setw(13) << odom_pose.y() << ", " <<
        //                  std::setw(13)
        //                  << odom_pose.heading() << "]" << endl;
        // std::cout.setf(std::ios::right, std::ios::adjustfield);
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
