#include <lwr_peg_in_hole/ellipse_detection.hpp>

void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info){
  
  /** Image processing to find the ellipses in the image **/
  // Original image
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg);
  if(debug_)
    cv::imshow("Original Image", cv_image->image);
  
  // MedianBlur to help edge detection
  cv::medianBlur(cv_image->image, cv_image->image, 11);
  if(debug_)
    cv::imshow("1/MedianBlur", cv_image->image);
  
  // Grayscale image
  cv::cvtColor(cv_image->image, cv_image->image, cv::COLOR_RGB2GRAY);
  if(debug_)
    cv::imshow("2/MonoImage", cv_image->image);
  
  // Adaptive threshold giving almost only edges
  cv::adaptiveThreshold(cv_image->image, cv_image->image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
  if(debug_)
    cv::imshow("3/AdaptiveThresholding", cv_image->image);
  
  // Opening to remove extra edges
  int morph_size = 1;
  cv::Mat morph_kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::morphologyEx(cv_image->image, cv_image->image, cv::MORPH_CLOSE, morph_kernel);
  if(debug_)
    cv::imshow("4/Opening", cv_image->image);
  
  cv::bitwise_not(cv_image->image, cv_image->image);
  
  // Find contours
  std::vector< std::vector<cv::Point> >  contours;
  cv::findContours( cv_image->image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );  
  if(debug_)
    std::cout << "There is " << contours.size() << " contours" <<std::endl;
  
  // Fit ellipses on the contours
  std::vector<cv::RotatedRect> ellipses;
  for(int i = 0; i < contours.size(); i++){
    // Contour should have a minimum pixel size
    if (contours[i].size() > contour_min_pix_size_ ){
      cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);
      // Ellipse are kept only if the ratio is high enough
      if ((ellipse.size.height > ellipse.size.width*(2 - ellipse_max_ratio_) ) || (ellipse.size.height < ellipse.size.width*ellipse_max_ratio_) )
        continue;
      ellipses.push_back(ellipse);
    }
  }
  if(debug_){
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
    std::cout << final_ellipses.size() << " ellipses remaining !!!!" << std::endl;
  
  /** Printing ellipses on the image **/
  cv::RNG rng(12345);
  cv_bridge::CvImagePtr cv_image_orig = cv_bridge::toCvCopy(image_msg);
  for(int i = 0; i < final_ellipses.size() ; i++){
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    cv::ellipse(cv_image_orig->image,final_ellipses[i], color, 1, 8);
  }
  cv::imshow("Ellipse detection", cv_image_orig->image);
  
  /** Convert the ellipse 2D to a circle 3D **/
  for(int i = 0; i< final_ellipses.size() ; i++){
    std::vector<geometry_msgs::Pose> holes_pose = ellipse_to_pose(final_ellipses[i], *cam_info, hole_radius_);
    if (holes_pose.size() > 0){
      geometry_msgs::PoseStamped pose_in, pose_converted;
      try{
        tf_listener_->waitForTransform(cam_info->header.frame_id, base_frame_, ros::Time(0.0), ros::Duration(1.0));
        pose_in.header.frame_id = cam_info->header.frame_id;
//         pose_in.header.stamp = ros::Time(0.0);
        pose_in.pose = holes_pose[0];
        tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
        pose_in.pose = holes_pose[1];
        tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        break;
      }
      if (debug_){
        for(int i = 0; i < holes_pose.size() ; i++)
          std::cout << "Position is : ("<<pose_converted.pose.position.x<<","<<pose_converted.pose.position.y<<","<<pose_converted.pose.position.z<<")"<<std::endl;
      }
      if (i==0){
        geometry_msgs::PoseStamped pose_stmp;
        pose_stmp.header.frame_id = base_frame_;
        pose_stmp.pose = pose_converted.pose;
        pose_pub_.publish(pose_stmp);
      }
    }
  }
  
  /** TODO Compare the poses to the estimation **/
  
  /** TODO Keep the closest, if close enough...or discard **/
  
  // Give some time in loop to display the images
  cv::waitKey(10);
}

std::vector<geometry_msgs::Pose> ellipse_to_pose(cv::RotatedRect ellipse, sensor_msgs::CameraInfo camera_info, double radius){
  
  std::vector<geometry_msgs::Pose> poses;
  cv::Mat_<double> Q, EigenVal, EigenVect;
  
  // Q symetrical matrix
  // V eigen vectors of Q
  // E eigen values of Q
  
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info);
  cv::Mat_<double> intrinsic(cam_model.intrinsicMatrix());
  
  double a = ellipse.size.width/2., b = ellipse.size.height/2.;
  double angle = M_PI/180.0 * (float) ellipse.angle;
  double ca = cos(angle), sa = sin(angle);
  double cx = intrinsic(0,2), cy = intrinsic(1,2);
  cv::Mat_<double>  Re = (cv::Mat_<double>(2,2) << ca, -sa, sa, ca);
  cv::Mat_<double> ABInvTAB = (cv::Mat_<double>(2,2) << 1./(a*a), 0., 0., 1./(b*b));
  cv::Mat_<double>  X0 = (cv::Mat_<double>(2,1) << ellipse.center.x-cx, ellipse.center.y-cy);
  cv::Mat_<double>  M = Re * ABInvTAB * Re.t();
  cv::Mat_<double>  Mf = X0.t() * M * X0;
  double A = M(0,0);
  double B = M(0,1);
  double C = M(1,1);
  double D = - A * X0(0) - B * X0(1);
  double E = - B * X0(0) - C * X0(1);
  double F = Mf(0,0) - 1.0;
  Q =  (cv::Mat_<double>(3,3) << A, B, D,
                                 B, C, E,
                                 D, E, F);
  
  double fx = intrinsic(0,0), fy = intrinsic(1,1);
  double focalLength = (fx+fy)/2.0;
  Q(0,0) = Q(0,0);
  Q(0,1) = Q(0,1);
  Q(0,2) = Q(0,2) / (focalLength);
  Q(1,0) = Q(1,0);
  Q(1,1) = Q(1,1);
  Q(1,2) = Q(1,2) / (focalLength);
  Q(2,0) = Q(2,0) / (focalLength);
  Q(2,1) = Q(2,1) / (focalLength);
  Q(2,2) = Q(2,2) / (focalLength*focalLength);

  cv::eigen(Q, EigenVal, EigenVect);
  EigenVect = EigenVect.t();
  // The three eigen values
  double e1 = EigenVal.at<double>(0);
  double e2 = EigenVal.at<double>(1);
  double e3 = EigenVal.at<double>(2);
  // The eight possibilities
  double S1[] = {+1,+1,+1,+1,-1,-1,-1,-1};
  double S2[] = {+1,+1,-1,-1,+1,+1,-1,-1};
  double S3[] = {+1,-1,+1,-1,+1,-1,+1,-1};

  // Two constant values depending on the eigen values
  double g = sqrt((e2-e3)/(e1-e3));
  double h = sqrt((e1-e2)/(e1-e3));
  
  cv::Point2d projections[2]; // Two plausible solutions
  cv::Point3d translations[2]; // Two plausible solutions
  cv::Vec3d normals[2]; // Two plausible solutions
  
  // reset the solutions to zero
  translations[0] = translations[1] = cv::Point3d(0,0,0);
  normals[0] = normals[1] = cv::Vec3d(0,0,0);
  projections[0] = projections[1] = cv::Point2d(0,0);
  
  // K is meant to select only 2 solutions (There can be only 2 anyway)
  unsigned int k = 0;
  
  for(int i = 0; i < 8; i++) {
      double z0 =  S3[i] * (e2 * radius) / sqrt(-e1*e3);
      double Tx =  S2[i] * e3/e2 * h;
      double Ty =  0.;
      double Tz = -S1[i] * e1/e2 * g;
      double Nx =  S2[i] * h;
      double Ny =  0.;
      double Nz = -S1[i] * g;

      cv::Mat_<double> t = (z0 * EigenVect * (cv::Mat_<double>(3,1) << Tx, Ty, Tz));
      cv::Mat_<double> n = (EigenVect *  (cv::Mat_<double>(3,1) << Nx, Ny, Nz));

      // identify the two possible solutions
      if((t(2) > 0) && (n(2)<0)) {
          if(k > 1) continue;
          translations[k] = cv::Point3d(t(0), t(1), t(2));
          normals[k] = cv::Vec3d(n(0), n(1), n(2));
          // Projection
          cv::Mat_<double> Pc = intrinsic * t;
          projections[k].x = Pc(0)/Pc(2);
          projections[k].y = Pc(1)/Pc(2);
          k++;
      }
  }
  
  for (int i= 0; i<2; i++){
    cv::Mat_<double> unitZ = (cv::Mat_<double>(3,1) << 0, 0, 1);
    cv::Mat_<double> nvec(normals[i]);
    nvec = nvec/cv::norm(nvec);
    cv::Mat_<double> c2 = nvec;
    cv::Mat_<double> c1 = unitZ.cross(c2);
    c1 = c1/cv::norm(c1);
    cv::Mat_<double> c0 = c1.cross(c2);
    c0 = c0/cv::norm(c0);
    
    tf::Matrix3x3 R;
    tf::Quaternion rotation;
    R.setValue(c0(0), c1(0), c2(0), c0(1), c1(1), c2(1), c0(2), c1(2), c2(2));
    R.getRotation(rotation);
    
    geometry_msgs::Pose pose;
    pose.position.x = translations[i].x;
    pose.position.y = translations[i].y;
    pose.position.z = translations[i].z;
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();
    pose.orientation.w = rotation.getW();
    poses.push_back(pose);
  }
  return poses;
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
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  // ROS suscribers and publishers
  image_transport::ImageTransport image_transport_(nh);
  image_transport::CameraSubscriber depth_sub_ = image_transport_.subscribeCamera(img_in_topic_, 1, callback);
  
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("hole_pose", 1);
  
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