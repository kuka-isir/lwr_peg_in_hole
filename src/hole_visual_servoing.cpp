#include <lwr_peg_in_hole/hole_visual_servoing.hpp>
#include <fstream>

HoleVisualServoing::HoleVisualServoing(ros::NodeHandle& nh) : nh_(nh)
{ 
  has_set_ideal_ = false;
  
  tf_listener_ = new tf::TransformListener();
  
  // Reading params
  ros::NodeHandle nh_priv("~");
  nh_priv.param<std::string>("img_in",img_in_topic_,"/head_cam/image_rect_color");
  nh_priv.param<int>("contour_min_pix_size",contour_min_pix_size_,50);
  nh_priv.param<double>("ellipse_max_ratio",ellipse_max_ratio_,0.75);
  nh_priv.param<double>("hole_radius",hole_radius_,0.0035);
  nh_priv.param<double>("holes_min_spacing",holes_min_spacing_,1);
  nh_priv.param<bool>("debug",debug_,false);
  nh_priv.param<std::string>("base_frame",base_frame_,"base_link");
  nh_priv.param<double>("fit_ellipse_max_error",fit_ellipse_max_error_,10.0);
  
  // ROS suscribers and publishers
  image_transport::ImageTransport image_transport_(nh_);
  depth_sub_ = image_transport_.subscribeCamera(img_in_topic_, 1, &HoleVisualServoing::callback, this);
  
  // OpenCV windows 
  if(debug_){
    cv::namedWindow("Original Image");
    cv::namedWindow("1/MedianBlur");
    cv::namedWindow("2/MonoImage");
    cv::namedWindow("3/AdaptiveThresholding");
    cv::namedWindow("4/Opening");
    cv::namedWindow("Ellipse detection");
  }
  
  seeing_ellipses_ = false;
}

bool HoleVisualServoing::isPinAboveHole()
{       
  std::cout << "Error in image ["<<std::abs(error_x_)<<","<<std::abs(error_y_)<<"]"<<std::endl;
  return ((std::abs(error_x_) < 1) && (std::abs(error_y_) < 1)) && seeing_ellipses_;
}

void HoleVisualServoing::saveHole()
{       

  YAML::Emitter out_yaml;
  out_yaml << YAML::BeginMap;
  out_yaml << YAML::Key << "angle";
  out_yaml << YAML::Value << ideal_hole_location_.angle;
  out_yaml << YAML::Key << "center_x";
  out_yaml << YAML::Value << ideal_hole_location_.center.x;
  out_yaml << YAML::Key << "center_y";
  out_yaml << YAML::Value << ideal_hole_location_.center.y;
  out_yaml << YAML::Key << "width";
  out_yaml << YAML::Value << ideal_hole_location_.size.width;
  out_yaml << YAML::Key << "height";
  out_yaml << YAML::Value << ideal_hole_location_.size.height;
  out_yaml << YAML::EndMap;

  std::string filename = ros::package::getPath("lwr_peg_in_hole")+"/holes/perfect_ellipse.yaml" ;
  std::ofstream fout(filename.c_str());
  fout << out_yaml.c_str();
  
  save_ideal_hole_location_ = false;
  has_set_ideal_ = false;
  
  return;
  
}

void HoleVisualServoing::loadSavedHole(std::string filename)
{
  ROS_INFO_STREAM("Trying to load saved ellipse");
  YAML::Node ellipse_config = YAML::LoadFile(ros::package::getPath("lwr_peg_in_hole")+"/holes/"+filename+".yaml");

  if (!ellipse_config["angle"]) {
    ROS_ERROR("Couldn't read ellipse in %s", filename.c_str());
    has_set_ideal_ = false;
    return;
  }
  YAML::Node angle = ellipse_config["angle"];
  ideal_hole_location_.angle = angle.as<float>();
  YAML::Node center_x = ellipse_config["center_x"];
  ideal_hole_location_.center.x = center_x.as<float>();
  YAML::Node center_y = ellipse_config["center_y"];
  ideal_hole_location_.center.y = center_y.as<float>();
  YAML::Node width = ellipse_config["width"];
  ideal_hole_location_.size.width = width.as<float>();
  YAML::Node height = ellipse_config["height"];
  ideal_hole_location_.size.height = height.as<float>();
  
  has_set_ideal_ = true;
}


bool HoleVisualServoing::verifyCircle(const cv::RotatedRect& ellipse_fitted, const cv::Point2f& min_circle_center, float& min_circle_radius){  
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

cv::RotatedRect HoleVisualServoing::closestEllipse(const std::vector<cv::RotatedRect>& ellipses){

  int best_ellipse = -1;
  double best_dist = -1.0;
  
  for(int i=0; i<ellipses.size(); i++){
    // Check on width
    if((ideal_hole_location_.size.height > ellipses[i].size.height +5) || (ideal_hole_location_.size.height < ellipses[i].size.height -5))
      continue;
    
    // Check on height
    if((ideal_hole_location_.size.width > ellipses[i].size.width +5) || (ideal_hole_location_.size.width < ellipses[i].size.width -5))
      continue;
    
    double current_dist = (ellipses[i].center.x-ideal_hole_location_.center.x)*(ellipses[i].center.x-ideal_hole_location_.center.x)
                        + (ellipses[i].center.y-ideal_hole_location_.center.y)*(ellipses[i].center.y-ideal_hole_location_.center.y);
                        
    if((best_dist <0) || (current_dist < best_dist)){
      best_dist = current_dist;
      best_ellipse = i;
    }
  }
  
  if(best_ellipse >=0){
    seeing_ellipses_ = true;
    return ellipses[best_ellipse];
  }else{
    seeing_ellipses_ = false;
    return cv::RotatedRect();
  }
}


void HoleVisualServoing::callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info){
  
  // Load perfect ellipse once
  if(!has_set_ideal_)
    loadSavedHole("perfect_ellipse");
  
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
    if(this->verifyCircle(ellipse, min_circle_center, min_circle_radius)){
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
    cv::RotatedRect printTargetellipse(ideal_hole_location_);
    printTargetellipse.size.height -=5;
    printTargetellipse.size.width -=5;
    cv::ellipse(cv_image_orig->image,printTargetellipse, color, 2, 8);
    printTargetellipse.size.height +=10;
    printTargetellipse.size.width +=10;
    cv::ellipse(cv_image_orig->image,printTargetellipse, color, 2, 8);
    
  }
  
  // Save current lonely ellipse if asked
  if(save_ideal_hole_location_ && final_ellipses.size() == 1)
  {
    ideal_hole_location_ = ellipses.back();
    saveHole();
    has_set_ideal_ = true; 
  }
  
  if(has_set_ideal_){
    seeing_ellipses_ = false;
    
    // Keep only the ellipse closest to the ideal
    cv::RotatedRect closest_ellipse = closestEllipse(final_ellipses);
    
    // Compute error
    if(seeing_ellipses_){
      // Print closest ellipse
      cv::Scalar green = cv::Scalar(0,255,0);
      cv::circle(cv_image_orig->image,cv::Point(closest_ellipse.center.x,closest_ellipse.center.y),4, green, -1, 8);
      cv::ellipse(cv_image_orig->image,closest_ellipse, green, 2, 8);
      
      error_x_ = ideal_hole_location_.center.x - closest_ellipse.center.x;
      error_y_ = ideal_hole_location_.center.y - closest_ellipse.center.y;
    }
    else{
      // Stop moving if not seeing an ideal like ellipse
      error_x_ = 0.0;
      error_y_ = 0.0;
    }
  }
  
  cv::imshow("Ellipse detection", cv_image_orig->image);
  
  // Give some time in loop to display the images
  cv::waitKey(10);
}
