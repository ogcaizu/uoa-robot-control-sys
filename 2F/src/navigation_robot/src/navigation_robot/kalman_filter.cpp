#include "../../include/navigation_robot.hpp"
//#include "../../include/Eigen/Core"
#include <math.h>

using Eigen::MatrixXd;

// double global_v = 0.0;
// double global_omega = 0.0;

void kalman_filter(bool QR_get, ecl::LegacyPose2D<double>& kalman_pose,
                   double& kalman_pose_heading,
                   ecl::LegacyPose2D<double> QR_pose, double QR_pose_heading,
                   double global_v, double global_omega, double delta_t,
                   double& kalman_dx)
{
  // bool est_heading_negative = false;

  //  cout
  //      << "============================== Kalman_filter
  //      ========================"
  //      << endl;
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  boost::posix_time::hours time_difference(9);
  my_posix_time += time_difference;
  //  std::string iso_time_str =
  //  boost::posix_time::to_simple_string(my_posix_time);
  //  cout << iso_time_str << endl;

  static MatrixXd P = MatrixXd::Identity(3, 3);

  static MatrixXd Q(3, 3);
  Q << 0.013889, 0, 0, 0, 0.013861, 0, 0, 0, 0.001398;

  static MatrixXd R(3, 3);
  R << 0.000449, 0, 0, 0, 0.00093586, 0, 0, 0, 0.000374;

  static MatrixXd u(2, 1);
  u(0, 0) = global_v;
  u(1, 0) = global_omega;
  //  std::cout << "u:[v,omega] =" << u.transpose() << "kalman_dx = " <<
  //  kalman_dx
  //            << std::endl;

  static MatrixXd x(3, 1);
  x(0, 0) = kalman_pose.x();
  x(1, 0) = kalman_pose.y();
  x(2, 0) = kalman_pose_heading; // Renge -2nPi ~ 2nPi

  static MatrixXd B(3, 2);
  B(0, 0) = delta_t * std::cos(kalman_pose_heading);
  B(0, 1) = 0.0;
  B(1, 0) = delta_t * std::sin(kalman_pose_heading);
  B(1, 1) = 0.0;
  B(2, 0) = 0.0;
  B(2, 1) = delta_t;
  // cout << "---B---" << endl<< B << endl;
  MatrixXd Bu = B * u;
  kalman_dx += std::sqrt(std::pow(Bu(0, 0), 2) + std::pow(Bu(1, 0), 2));
  x = x + Bu; // eq1

  // if(x(2,0)<0){
  // est_heading_negative = true;
  // x(2,0) = std::abs(x(2,0));
  //}

  //  cout << "-- x-^ --" << endl
  //       << x << endl;

  P = P + Q; // eq2
             //  cout << "-- P- --" << endl
             //       << P << endl;

  if (QR_get)
  {
    //    cout << "---------- QR get!! ----------" << endl;

    static MatrixXd z(3, 1);
    z(0, 0) = QR_pose.x();
    z(1, 0) = QR_pose.y();
    z(2, 0) = QR_pose_heading; // old std::abs(QR_pose.heading());
    //    cout << "-- Z --" << endl
    //         << z << endl;

    MatrixXd y = z - x;
    MatrixXd S = R + P;
    MatrixXd K = P * S.inverse();
    x = x + K * y;
    //    cout << "-- x+^ --" << endl
    //         << x << endl;

    MatrixXd I_K = Eigen::Matrix3d::Identity() - K;
    P = I_K * P * I_K.transpose() + (K * R * K.transpose());
    //    cout << "-- P+ --" << endl
    //         << P << endl;
  }

  kalman_pose.x(x(0, 0));
  kalman_pose.y(x(1, 0));
  kalman_pose_heading = x(2, 0);

  // if(est_heading_negative == true)
  //  kalman_pose.heading(-x(2, 0));
  // else
  //  kalman_pose.heading(x(2,0));

  //  cout << "---x^---" << endl
  //       << x << endl;
}
