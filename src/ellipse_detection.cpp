#include <lwr_peg_in_hole/ellipse_detection.hpp>

bool verifyCircle(const cv::RotatedRect& ellipse_fitted, const cv::Point2f& min_circle_center, float& min_circle_radius){  
  // Check if ellipse is close enough to a circle
  if ( ellipse_fitted.size.height > ellipse_fitted.size.width*(2 - ellipse_max_ratio_) || ellipse_fitted.size.height < ellipse_fitted.size.width*ellipse_max_ratio_)
    return false;
  
  // Check if ellipse radius is big enough
  if ( std::min(ellipse_fitted.size.height, ellipse_fitted.size.width) < 25 )
    return false;

  // Compute rms with circles position and radius
  double rms = std::sqrt( std::pow(ellipse_fitted.center.x - min_circle_center.x,2) 
                        + std::pow(ellipse_fitted.center.y - min_circle_center.y,2) 
                        + std::pow(min_circle_radius*2.0 - std::min(ellipse_fitted.size.width, ellipse_fitted.size.height),2) )
                        /std::sqrt(3.0);
  
  if(debug_)
    std::cout << "RMS : " << rms << std::endl;
    
  return rms<fit_ellipse_max_error_;
}

void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info){
  
  /** Image processing to find the ellipses in the image **/
  // Original image
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg);
  if(debug_)
    cv::imshow("Original Image", cv_image->image);
  
  // Mask definition
  int width_mask = 1360, height_mask = 750;
  cv::Mat mask( cv_image->image.size(), CV_8UC1, cv::Scalar(0,0,0));
  cv::rectangle(mask, cv::Point(1360/2-width_mask/2, 1024-height_mask), cv::Point(1360/2+width_mask/2, 1024), cv::Scalar(255,255,255), CV_FILLED);
  
  // Apply ROI  
  cv::Mat applied_mask;
  cv_image->image.copyTo(applied_mask, mask);
  cv_image->image = applied_mask;
  
  // Grayscale image
  cv::cvtColor(cv_image->image, cv_image->image, cv::COLOR_RGB2GRAY);
  if(debug_)
    cv::imshow("2/MonoImage", cv_image->image);
  
  // Equalize histogram
  cv::equalizeHist(cv_image->image,cv_image->image);
  
  // MedianBlur to help edge detection
  cv::medianBlur(cv_image->image, cv_image->image, 11);
    if(debug_)
    cv::imshow("1/MedianBlur", cv_image->image);

  // Adaptive threshold giving almost only edges
  cv::adaptiveThreshold(cv_image->image, cv_image->image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
  if(debug_)
    cv::imshow("3/AdaptiveThresholding", cv_image->image);
  
  // Opening to remove extra edges
  int morph_size = 1;
  cv::Mat morph_kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::morphologyEx(cv_image->image, cv_image->image, cv::MORPH_OPEN, morph_kernel);
  if(debug_)
    cv::imshow("4/Opening", cv_image->image);
  
  // Apply ROI on edge detection
  cv_image->image.copyTo(applied_mask, mask);
  cv_image->image = applied_mask;
  
  // Find contours
  std::vector< std::vector<cv::Point> >  contours;
  cv::findContours( cv_image->image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);  
  if(debug_)
    std::cout << "There is " << contours.size() << " contours" <<std::endl;
  
  cv::Mat big_contours( cv_image->image.size(), CV_8UC1, cv::Scalar(255,255,255));
  cv::Mat ellipse_n_area_contours;
  if(debug_){
    cv_bridge::CvImagePtr orig_bis = cv_bridge::toCvCopy(image_msg);
    ellipse_n_area_contours = orig_bis->image;
  }

  // Keep only ellipses
  std::vector<cv::RotatedRect> ellipses;
  for(int i = 0; i < contours.size(); i++){
    // Contour should have a minimum pixel size
    if(contours[i].size() < contour_min_pix_size_) 
      continue;
    
    // Fit ellipse on the contour
    cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);

    // Contour min enclosing circle
    cv::Point2f min_circle_center;
    float min_circle_radius;
    cv::minEnclosingCircle(contours[i],min_circle_center, min_circle_radius);

    // Check if the ellipse is close to the enclosing circle
    if(verifyCircle(ellipse, min_circle_center, min_circle_radius)){
      ellipses.push_back(ellipse);     
      if(debug_){
        cv::drawContours(ellipse_n_area_contours, contours, i, cv::Scalar(0,0,0));
        cv::ellipse(ellipse_n_area_contours, ellipse, cv::Scalar(255,0,0));
        cv::circle(ellipse_n_area_contours, min_circle_center, min_circle_radius, cv::Scalar(0,255,0));  
      }
    }
    if(debug_)
      cv::drawContours(big_contours, contours, i, cv::Scalar(0,0,0));
  }
  
  if(debug_){
    cv::imshow("BIG CONTOURS", big_contours);
    cv::imshow("ELLIPSE/MIN_RECT_AREA/MIN_ENCLOSING_CIRCLE", ellipse_n_area_contours);

    std::cout << "There is " << ellipses.size() << " ellipses" <<std::endl;
    for(int i = 0; i < ellipses.size() ; i++)
      std::cout
      <<"( x = " << ellipses[i].center.x <<" , y = " << ellipses[i].center.y 
      << " , width = "<< ellipses[i].size.width<< " , height = "<< ellipses[i].size.height
      << " , angle = "<< ellipses[i].angle<< " )"
      << std::endl;
  }
  
  /** Removing ellipses that are too close **/
  std::vector <int> to_remove;
  for(int ellipse_nb = 0; ellipse_nb < ellipses.size(); ellipse_nb++){
    for(int i = ellipse_nb; i < ellipses.size(); i++){
      if(i != ellipse_nb){
        if (std::sqrt(cv::norm(ellipses[i].center - ellipses[ellipse_nb].center)) < holes_min_spacing_){
          if(ellipses[i].size.width > ellipses[ellipse_nb].size.width)
            to_remove.push_back(ellipse_nb);
          else
            to_remove.push_back(i);
        }
      }
    }
  }
  std::vector<bool> keep_ellipses;
  keep_ellipses.resize(ellipses.size());
  for(int i = 0; i < keep_ellipses.size(); i++){
    keep_ellipses[i] = true;
  }
  for(int i = 0; i < to_remove.size(); i++){
    keep_ellipses[to_remove[i]] = false;
  }
  std::vector<cv::RotatedRect> final_ellipses;
  for(int i = 0; i<keep_ellipses.size(); i++){
    if (keep_ellipses[i])
      final_ellipses.push_back(ellipses[i]);
  }
  
  if(debug_)
  {
    std::cout << final_ellipses.size() << " ellipses remaining !!!!" << std::endl;
    std::cout << "There is " << final_ellipses.size() << " final_ellipses" <<std::endl;
    for(int i = 0; i < final_ellipses.size() ; i++)
      std::cout
      <<"( x = " << final_ellipses[i].center.x <<" , y = " << final_ellipses[i].center.y 
      << " , width = "<< final_ellipses[i].size.width<< " , height = "<< final_ellipses[i].size.height
      << " , angle = "<< final_ellipses[i].angle<< " )"
      << std::endl;
  }
  
  /** Printing ellipses on the image **/
  cv::RNG rng(12345);
  cv_bridge::CvImagePtr cv_image_orig = cv_bridge::toCvCopy(image_msg);
  for(int i = 0; i < final_ellipses.size() ; i++){
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    cv::circle(cv_image_orig->image,cv::Point(final_ellipses[i].center.x,final_ellipses[i].center.y),4, color, -1, 8);
    cv::ellipse(cv_image_orig->image,final_ellipses[i], color, 1, 8);
    
    color = cv::Scalar(0,0,255);
    cv::circle(cv_image_orig->image,cv::Point(ideal_hole_location_.center.x,ideal_hole_location_.center.y),4, color, -1, 8);
    cv::ellipse(cv_image_orig->image,ideal_hole_location_, color, 1, 8);
  }
  cv::imshow("Ellipse detection", cv_image_orig->image);
  
  bool save_ideal_hole_location = false;
  ros::param::get("~save_ideal_hole_location",save_ideal_hole_location);
  if(save_ideal_hole_location && final_ellipses.size() == 1)
  {
    ros::param::set("~save_ideal_hole_location",false);
    ideal_hole_location_ = ellipses.back();
    has_set_ideal_ = true;
    
  }
  else if(final_ellipses.size() > 1)
  {
    ROS_ERROR("There's more than one ellipse in sight !");
  }
  else if(final_ellipses.size() == 0)
  {
    ROS_ERROR("No ellipse found !");
  }
  
  
  // Compute errors
  if(final_ellipses.size() == 1 && has_set_ideal_)
  {
    cv::RotatedRect ellipse_current = ellipses.back();
    double error_x = ideal_hole_location_.center.x - ellipse_current.center.x;
    double error_y = ideal_hole_location_.center.y - ellipse_current.center.y;
    geometry_msgs::Twist error;
    error.linear.x = error_x;
    error.linear.y = error_y;
    error_pub_.publish(error);
    
//     /** Convert the ellipse 2D to a circle 3D **/
//     std::vector<geometry_msgs::Pose> holes_pose = ellipse_to_pose(ellipses.back(), *cam_info, hole_radius_);
//     if (holes_pose.size() > 0){
//       geometry_msgs::PoseStamped pose_in, pose_converted;
//       try{
//         tf_listener_->waitForTransform(cam_info->header.frame_id, base_frame_, ros::Time(0.0), ros::Duration(1.0));
//         pose_in.header.frame_id = cam_info->header.frame_id;
//         pose_in.pose = holes_pose[0];
//         tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
//         pose0_pub_.publish(pose_converted);
//         pose_in.pose = holes_pose[1];
//         tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
//         pose1_pub_.publish(pose_converted);       
//       }
//       catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what());}
//       
//       if (debug_){
//         for(int i = 0; i < holes_pose.size() ; i++)
//           std::cout << "Position is : ("<<pose_converted.pose.position.x<<","<<pose_converted.pose.position.y<<","<<pose_converted.pose.position.z<<")"<<std::endl;
//       }
//     }
  }
  
  /** TODO Compare the poses to the estimation **/
  
  /** TODO Keep the closest, if close enough...or discard **/
  
  // Give some time in loop to display the images
  cv::waitKey(10);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipse_detection_node");
  ros::NodeHandle nh, nh_priv("~");
  tf_listener_ = new tf::TransformListener();
  
  // Reading params
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("img_in",img_in_topic_);
  params_loaded *= nh_priv.getParam("contour_min_pix_size",contour_min_pix_size_);
  params_loaded *= nh_priv.getParam("ellipse_max_ratio",ellipse_max_ratio_);
  params_loaded *= nh_priv.getParam("hole_radius",hole_radius_);
  params_loaded *= nh_priv.getParam("holes_min_spacing",holes_min_spacing_);
  params_loaded *= nh_priv.getParam("debug",debug_);
  params_loaded *= nh_priv.getParam("base_frame",base_frame_);
  params_loaded *= nh_priv.getParam("out_pose_topic",out_pose_topic_);
  params_loaded *= nh_priv.getParam("out_error_topic",out_error_topic_);
  params_loaded *= nh_priv.getParam("fit_ellipse_max_error",fit_ellipse_max_error_);
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  // ROS suscribers and publishers
  image_transport::ImageTransport image_transport_(nh);
  image_transport::CameraSubscriber depth_sub_ = image_transport_.subscribeCamera(img_in_topic_, 1, callback);
  
//   pose0_pub_ = nh.advertise<geometry_msgs::PoseStamped>(out_pose_topic_+"_0", 1);
//   pose1_pub_ = nh.advertise<geometry_msgs::PoseStamped>(out_pose_topic_+"_1", 1);
  error_pub_ = nh_priv.advertise<geometry_msgs::Twist>(out_error_topic_, 1);
  
  // OpenCV windows 
  if(debug_){
    cv::namedWindow("Original Image");
    cv::namedWindow("1/MedianBlur");
    cv::namedWindow("2/MonoImage");
    cv::namedWindow("3/AdaptiveThresholding");
    cv::namedWindow("4/Opening");
  }
  cv::namedWindow("Ellipse detection");
  
  ros::spin();
  ros::shutdown();
  return 0;
}