#include "ros/ros.h"
// #include "duckietown_msgs/GetGroundCoord.h"
// #include "duckietown_msgs/EstimateHomography.h"
#include "ground_projection/GetGroundCoord.h"
#include "ground_projection/EstimateHomography.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/highgui.hpp>

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <opencv/cv.h>

// #include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

bool estimate_homography(ground_projection::EstimateHomography::Request &req, 
                         ground_projection::EstimateHomography::Response &res)
{
  ROS_INFO("inside estimate_homography");

  // cv_bridge
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    // cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
    std::cout << "1" << std::endl;
    cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::MONO8);
    std::cout << "2" << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }


  // // estimate homography
  // int board_w = 8;
  // int board_h = 6;
  // int board_n = board_w * board_h;
  // cv::Mat Ir = cv_ptr->image;
  // float square_size = 0.025; // 2.5 cm // TODO: make it param

  // cv::Size board_size(board_w, board_h);
  // std::vector<cv::Point2f> corners_;
  // bool found = findChessboardCorners(Ir, board_size, corners_);
  // if(found)
  // {
  //   cornerSubPix(Ir, corners_, cv::Size(11, 11), cv::Size(-1, -1),
  //     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  //   cv::Mat Irc;
  //   cvtColor(Ir, Irc, CV_GRAY2BGR);

  //   drawChessboardCorners(Irc, board_size, cv::Mat(corners_), found);
  //   imshow("image", Irc);
  // }
  // else
  // {
  //   std::cout << "Couldn't aquire a checkerboard" << std::endl;
  //   return false;
  // }

  // std::vector<cv::Point2f> pts_gnd_, pts_img_;
  // pts_gnd_.resize(board_w*board_h);
  // pts_img_.resize(board_w*board_h);

  // for(int r=0; r<board_h; r++)
  // {
  //     for(int c=0; c<board_w; c++)
  //     {
  //         pts_gnd_[r*(board_w)+c] = cv::Point2f(float(c)*square_size, float(r)*square_size);
  //         pts_img_[r*(board_w)+c] = corners_[r*(board_w)+c];
  //     }
  // }

  // cv::Mat H_ = cv::findHomography(pts_img_, pts_gnd_, CV_RANSAC);
  // H_.convertTo(H_, CV_32F);
  // std::cout << "H_: " << H_ << std::endl;


  // req.image;

  // cv::imshow("image", cv_ptr->image);
  // cv::waitKey(0);

  // cvNamedWindow("image");
  // IplImage iplimg = cv_ptr->image;
  // cvShowImage("image", &iplimg);
  // cv::waitKey(0);

  std::cout << "before return" << std::endl;

  return true;
}

bool get_ground_coordinate(ground_projection::GetGroundCoord::Request &req, 
                           ground_projection::GetGroundCoord::Response &res)
{
  ROS_INFO("request: u=%d, v=%d", req.uv.u, req.uv.v);
  res.xy.x = (float)req.uv.u;
  res.xy.y = (float)req.uv.v;  
  ROS_INFO("request: x=%f, y=%f", res.xy.x, res.xy.y);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_projection_server");

  ros::NodeHandle n;

  ros::ServiceServer service1 = n.advertiseService("estimate_homography", estimate_homography);
  ros::ServiceServer service2 = n.advertiseService("get_ground_coordinate", get_ground_coordinate);

  ROS_INFO("estimate_homography is ready.");
  ROS_INFO("get_ground_coordinate is ready.");
  
  cv::namedWindow("image");

  ros::spin();

  cv::destroyWindow("image");

  return 0;
}