#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "../Eigen/Eigen/QR"
#include <std_msgs/Int16.h>


using namespace cv;
using namespace std;
using namespace Eigen;

static const std::string OPENCV_WINDOW = "Image Window";
static const double frametime = 1;
class laneDetection{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher steer_pub_;
  ros::Publisher speed_pub_;
  public:
    laneDetection()
    : it_(nh_)
    {
      // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1,&laneDetection::image_processing, this);
    steer_pub_ = nh_.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
    speed_pub_ = nh_.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  
    //cv::namedWindow(OPENCV_WINDOW);
    }

    ~laneDetection()
    {
      //cv::destroyWindow(OPENCV_WINDOW);
    }
    // The input argument is the location of the video
      //cv::VideoCapture video_capture(0);
      
      // if (!video_capture.isOpened()){
      //   cout << "can not open camera" << endl;
      // }
    void image_processing(const sensor_msgs::ImageConstPtr& msg){
      static double Kp =500, Ki = 0.05, Kd = 1.5, cte = 0, p_error = 0, i_error = 0, d_error = 0,lastcte = 0;

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        int iLowH = 50;
        int iHighH = 70;

        int iLowS = 20; 
        int iHighS = 255;

        int iLowV = 100;
        int iHighV = 255;
/*
        int iLowH = 155;
        int iHighH = 175;

        int iLowS = 50; 
        int iHighS = 255;

        int iLowV = 100;
        int iHighV = 255;
*/


        //Create trackbars in "Control" window
        /*cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);*/


        Mat imgOriginal = cv_ptr->image;

        // cout << "Width : " << imgOriginal.size().width << endl;
        // cout << "Height: " << imgOriginal.size().height << endl;

        Rect myROI(1000, 630, 500, 450);

        Mat croppedImage = imgOriginal(myROI);

        Mat imgHSV;

        cvtColor(croppedImage, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        
        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", croppedImage); //show the original image
        
          ////////////////////////////////////////////////////////////
        std::vector<cv::Point> lane_points;
        findNonZero(imgThresholded, lane_points);
        // Eigen::VectorXd xvals(lane_points.size());
        // Eigen::VectorXd yvals(lane_points.size());
        std::vector<double> x_values, y_values, coeff;
        for(int i = 0; i <= lane_points.size(); i++){
              // xvals(i) = lane_points[i].x;
              // yvals(i) = lane_points[i].y;
              x_values.push_back(lane_points[i].x);
              y_values.push_back(lane_points[i].y);
        }
        //auto coeff = polyfit(yvals,xvals,2);
        polyfit(y_values, x_values, coeff, 2);
        // cout << "a0: " << coeff[0] << endl;
        // cout << "a1: " << coeff[1] << endl;
        // cout << "a2: " << coeff[2] << endl;
        int heigth = 5;
        int width = 10;
        for (int y = heigth; y <= imgThresholded.size().height-heigth; y += heigth) {
           int x = coeff[0] + coeff[1]*y + coeff[2]*y*y;
           cv::rectangle(croppedImage,Point(x-width,y-heigth),Point(x+width,y+heigth),cv::Scalar(0, 255, 0));
        }
        //imshow("Original", croppedImage); 
        //imshow("Thresholded Image", imgThresholded);

        int y0 = imgThresholded.size().height;
        int x0 = coeff[0] + coeff[1]*y0 + coeff[2]*y0*y0;
        int y1 = imgThresholded.size().height - 100;
        int x1 = coeff[0] + coeff[1]*y1 + coeff[2]*y1*y1;
        double tang = (x1-x0)/(y1-y0);
        cout << "tang: " << tang << endl;
        /////////////////////////////////////////////////////////////7
        cte = -tang;
        i_error += cte*frametime;
        d_error = (cte -lastcte)/frametime;
        std_msgs::Int16 steer;
        steer.data = (Kp*cte + Ki*i_error + Kd*d_error);
        lastcte = cte;
        steer_pub_.publish(steer);


        std_msgs::Int16 speed;  
        speed.data = 200;

        speed_pub_.publish(speed);

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            return; 
        }
        }
        //////////////////////////////////////////////////////////////////////////////////
//         Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) 
// {
//   assert(xvals.size() == yvals.size());
//   assert(order >= 1 && order <= xvals.size() - 1);
//   Eigen::MatrixXd A(xvals.size(), order + 1);

//   for (int i = 0; i < xvals.size(); i++) {
//     A(i, 0) = 1.0;
//   }

//   for (int j = 0; j < xvals.size(); j++) {
//     for (int i = 0; i < order; i++) {
//       A(j, i + 1) = A(j, i) * xvals(j);
//     }
//   }

//   auto Q = A.householderQr();
//   auto result = Q.solve(yvals);
//   return result;
// }

void polyfit(const std::vector<double> &xv, const std::vector<double> &yv, std::vector<double> &coeff, int order)
{
	Eigen::MatrixXd A(xv.size(), order+1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	Eigen::VectorXd result;

	assert(xv.size() == yv.size());
	assert(xv.size() >= order+1);

	// create matrix
	for (size_t i = 0; i < xv.size(); i++)
	for (size_t j = 0; j < order+1; j++)
		A(i, j) = pow(xv.at(i), j);

	// solve for linear least squares fit
	result = A.householderQr().solve(yv_mapped);

	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
};
/////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Lane_Detection_Test");
  laneDetection laneDetection_object;
  ros::spin();
  ROS_INFO("Lane Detection tested!");
  return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
