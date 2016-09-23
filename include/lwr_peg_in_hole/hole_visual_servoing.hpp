#ifndef HOLE_VISUAL_SERVOING_HPP
#define HOLE_VISUAL_SERVOING_HPP

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#define M_PI 3.14159265358979323846  /* pi */

class HoleVisualServoing
{
public:
  HoleVisualServoing(ros::NodeHandle& nh);

  void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
  
  bool verifyCircle(const cv::RotatedRect& ellipse_fitted, const cv::Point2f& min_circle_center, float& min_circle_radius); 
  
  bool isPinAboveHole();
  
  void saveHole();

  ros::NodeHandle& nh_;
  tf::TransformListener* tf_listener_;
  
  std::string img_in_topic_, base_frame_;
  int contour_min_pix_size_;
  double ellipse_max_ratio_, hole_radius_, holes_min_spacing_,fit_ellipse_max_error_;
  cv::RotatedRect ideal_hole_location_;
  bool has_set_ideal_, debug_, save_ideal_hole_location_;
  double error_x_, error_y_;
  image_transport::CameraSubscriber depth_sub_;

};

#endif
