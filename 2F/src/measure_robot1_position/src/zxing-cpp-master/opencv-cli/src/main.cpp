/*
 *  Copyright 2010-2011 Alessandro Francescon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <measure_robot1_position/Position.h>
#include <sstream>
#include <string>
#include <exception>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/MatSource.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"


#define PI 3.141592
#define Ku 0.002881
#define Kv 0.002808

using namespace std;
using namespace zxing;
using namespace zxing::qrcode;
using namespace cv;

typedef struct vector {
  float x;
  float y;
}vec;

typedef struct point {
  float x;
  float y;
}point;

void printUsage(char** argv) {

  // Print usage
  cout << "Usage: " << argv[0] << " [-d <DEVICE>] [-w <CAPTUREWIDTH>] [-h <CAPTUREHEIGHT>]" << endl
       << "Read QR code from given video device." << endl
       << endl;

}

Point toCvPoint(Ref<ResultPoint> resultPoint) {
  return Point(resultPoint->getX(), resultPoint->getY());
}

std::vector<string> split(std::string str, char del) {
  int first = 0;
  int last = str.find_first_of(del);

  std::vector<std::string> result;

  while (first < str.size()) {
    std::string subStr(str, first, last - first);

    result.push_back(subStr);

    first = last + 1;
    last = str.find_first_of(del, first);

    if (last == std::string::npos) {
      last = str.size();
    }
  }
  return result;
}


float getMid(float point1, float point2){
  float mid_u = (point1 + point2) / 2;
  return mid_u;
}


float searchMidDist(Ref<ResultPoint> point1, Ref<ResultPoint> point2){
  float point1_u = point1->getX();
  float point1_v = point1->getY();
  float point2_u = point2->getX();
  float point2_v = point2->getY();
  float mid_u = getMid(point1_u, point2_u);
  float mid_v = getMid(point1_v, point2_v);
  return sqrt((mid_u - 320)*(mid_u - 320) + (mid_v - 240)*(mid_v - 240));
}

int searchDirection(point center, point base_point, point mid_vec){
  float S = (base_point.x - center.x) * (mid_vec.y - center.y) - (base_point.y - center.y) * (mid_vec.x - center.x);
  if (S > 0)	return -1;
  else    return 1;
}

float searchAngle(point center, point base_point, point mid_vec, vec base, vec ob){
  float innar_product = base.x * ob.x + base.y * ob.y;
  float a = sqrt((base_point.x - center.x)*(base_point.x - center.x) + (base_point.y - center.y)*(base_point.y - center.y));
  float b = sqrt((mid_vec.x - center.x)*(mid_vec.x - center.x) + (mid_vec.y - center.y)*(mid_vec.y - center.y));
  float cos_th = innar_product / (a * b);
  float rad = acos(cos_th);
  float theta = rad / PI * 180;
  theta *= searchDirection(center, base_point, mid_vec);
  return theta;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "measure_robot1_position");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<measure_robot1_position::Position>("measure_robot1_position", 100);
  ros::Rate loop_rate(30);
  measure_robot1_position::Position msg;

  int deviceId = 0;
  int captureWidth = 640;
  int captureHeight = 480;
  char str[60];
  string text;
  //int counter = 1;
  bool multi = false;


  for (int j = 0; j < argc; j++) {
    // Get arg
    string arg = argv[j];

    if (arg.compare("-d") == 0) {

      if ((j + 1) < argc) {
	// Set device id
	deviceId = atoi(argv[++j]);
      } 

	else {
	// Log
	cerr << "Missing device id after -d" << endl;
	printUsage(argv);
	return 1;
      }

    } 

    else if (arg.compare("-w") == 0) {

      if ((j + 1) < argc) {
	// Set capture width
	captureWidth = (double)atoi(argv[++j]);
      } 

      else {
	// Log
	cerr << "Missing width after -w" << endl;
	printUsage(argv);
	return 1;
      }

    } 

    else if (arg.compare("-h") == 0) {

      if ((j + 1) < argc) {
	// Set capture height
	captureHeight = (double)atoi(argv[++j]);
      } 

      else {
	// Log
	cerr << "Missing height after -h" << endl;
	printUsage(argv);
	return 1;
      }

    } 

      else if (arg.compare("-m") == 0) {
      // Set multi to true
      multi = true;
    }

  }

  // Log
  cout << "Capturing from device " << deviceId << "..." << endl;

  // Open video captire
  VideoCapture videoCapture(deviceId);

  if (!videoCapture.isOpened()) {
    // Log
    cerr << "Open video capture failed on device id: " << deviceId << endl;
    return -1;
  }

  if (!videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, captureWidth)) {
    // Log
    cerr << "Failed to set frame width: " << captureWidth << " (ignoring)" << endl;
  }

  if (!videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, captureHeight)) {
    // Log
    cerr << "Failed to set frame height: " << captureHeight << " (ignoring)" << endl;
  }


  // The captured image and its grey conversion
  Mat image, grey, tmat;

  // Open output window
  namedWindow("QRDecorder", cv::WINDOW_AUTOSIZE);

  // Stopped flag will be set to -1 from subsequent wayKey() if no key was pressed
  int stopped = -1;

  // Set center
  int center_x = captureWidth / 2;
  int center_y = captureHeight / 2;


  while (ros::ok()) {
    // Search time
    clock_t start = clock();

    // Capture image
    bool result = videoCapture.read(image);

    if (result) {
      // Convert to grayscale
      cvtColor(image, grey, CV_BGR2GRAY);


      try {
	// Create luminance  source
	Ref<LuminanceSource> source = MatSource::create(grey);
	// Search for QR code
	Ref<Reader> reader;

	if (multi) {
	  reader.reset(new MultiFormatReader);
	}
	else {
	  reader.reset(new QRCodeReader);
	}

	Ref<Binarizer> binarizer(new GlobalHistogramBinarizer(source));
	Ref<BinaryBitmap> bitmap(new BinaryBitmap(binarizer));
	Ref<Result> result(reader->decode(bitmap, DecodeHints(DecodeHints::TRYHARDER_HINT)));

	// Get result point count
	int resultPointCount = result->getResultPoints()->size();

	for (int j = 0; j < 3; j++) {
	  // Draw circle
	  circle(image, toCvPoint(result->getResultPoints()[j]), 10, Scalar(110, 220, 0), 2);
	}



	// Draw boundary on image
	if (resultPointCount > 1) {

	  for (int j = 0; j < 3; j++) {
	    // Get start result point
	    Ref<ResultPoint> previousResultPoint = (j > 0) ? result->getResultPoints()[j - 1] : result->getResultPoints()[2];
	    // Draw line
	    line(image, toCvPoint(previousResultPoint), toCvPoint(result->getResultPoints()[j]), Scalar(110, 220, 0), 2, 8);
	    // Update previous point
	    previousResultPoint = result->getResultPoints()[j];
	  }

	}


	// Transformation matrx of image
	float mid_u = (int)getMid(result->getResultPoints()[0]->getX(), result->getResultPoints()[2]->getX());
	float mid_v = (int)getMid(result->getResultPoints()[0]->getY(), result->getResultPoints()[2]->getY());
	int delta_u = center_x - mid_u;
	int delta_v = center_y - mid_v;

	Mat mat = (Mat_<double>(2, 3)<< 1.0, 0.0, delta_u, 0.0, 1.0, delta_v);
	warpAffine(image, tmat, mat, tmat.size());

	// Search Theta
	point mid_vec = { delta_u + getMid(result->getResultPoints()[1]->getX(), result->getResultPoints()[2]->getX()), delta_v + getMid(result->getResultPoints()[1]->getY(), result->getResultPoints()[2]->getY()) };
	point base_point = { center_x, 0 } ;
	point center = { center_x, center_y };
	vec base = { base_point.x - center.x,  base_point.y - center.y };
	vec ob = { mid_vec.x - center_x, mid_vec.y - center_y };
	float theta = searchAngle(center, base_point, mid_vec, base, ob);
	float rad = theta * PI / 180;


	// Search X and Y
	float real_x, real_y, decode_x, decode_y,temp_u,temp_v,temp_x,temp_y,xr,yr;
	
	// To rectangular coordinates system from Image coordinate system
	temp_x = mid_v - captureHeight / 2;
	temp_y = captureWidth / 2 - mid_u;

	// Coordinate rotation
	xr = temp_x*cos(rad) - temp_y*sin(rad);
	yr = temp_x*sin(rad) + temp_y*cos(rad);

	// To Image coordinate system from rectangular coordinates system
	temp_u = captureWidth / 2 - yr;
	temp_v = captureHeight / 2 + xr; 

	delta_u = center_x - temp_u;
	delta_v = center_y - temp_v;
	string text_x, text_y;
	std::vector<string> sp_text;
	float delta_x = Ku * delta_u;
	float delta_y = Kv * delta_v;
//	cout << delta_x << "   " << delta_y << endl;
	text = result->getText()->getText();
	sp_text = split(text, ',');
	decode_x = atof(sp_text[0].c_str());
	decode_y = atof(sp_text[1].c_str());
	real_x = decode_x + delta_x;
	real_y = decode_y + delta_y;


	// Send to position as msg
//        cout << " " << endl;
//	cout << "Decode = " << result->getText()->getText() << endl;
	msg.x = real_x;
	msg.y = real_y;
  msg.theta = rad;
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        boost::posix_time::hours time_difference(9);
        my_posix_time += time_difference;
        std::string iso_time_str =
            boost::posix_time::to_simple_string(my_posix_time);
        msg.time = iso_time_str;
//	ROS_INFO("x = %f", msg.x);
//	ROS_INFO("y = %f", msg.y);
//	ROS_INFO("theta = %f", msg.theta);
	pub.publish(msg);

        cout << iso_time_str<<", " << real_x << ", " << real_y << ", " << theta << endl;


	if (resultPointCount > 0) {
	  // Draw text
	  putText(image, result->getText()->getText(), toCvPoint(result->getResultPoints()[0]), FONT_HERSHEY_PLAIN, 3, Scalar(255, 255, 255), 2);
	}



      }
      catch (const ReaderException& e) {
	//cerr << e.what() << " (1)" << endl;
      }
      catch (const zxing::IllegalArgumentException& e) {
	//cerr << e.what() << " (2)" << endl;
      }
      catch (const zxing::Exception& e) {
	//cerr << e.what() << " (3)" << endl;
      }
      catch (const std::exception& e) {
	//cerr << e.what() << " (4)" << endl;
      }


      // Show captured image
      imshow("QRDecorder", image);


      // Wait a key for 1 millis
      stopped = waitKey(1);

    } else {

      // Log
      cerr << "video capture failed" << endl;

    }
    loop_rate.sleep();
   
    /*clock_t end = clock();
    cout << "start = " << start << "  " << "end = " << end << endl;
    cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec." << endl;
    cout << endl;*/
  }

  // Release video capture
  videoCapture.release();

  return 0;

}
