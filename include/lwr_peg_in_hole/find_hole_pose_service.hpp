//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jimmy Da Silva, jimmy.dasilva@isir.upmc.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software. You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef FIND_HOLE_POSE_SERVICE_HPP
#define FIND_HOLE_POSE_SERVICE_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/graph/graph_concepts.hpp>

#include <lwr_peg_in_hole/FindHolePoseService.h>

class FindHolePoseService
{
  public:
    // Constructor.
    FindHolePoseService() : nh_(), nh_priv("~"), image_transport_(nh_){
      // Reading params
      nh_.param<std::string>("img_in", img_in_topic_, "/head_cam/image_rect_color");
      nh_.param<int>("contour_min_pix_size", contour_min_pix_size_, 50);
      nh_.param<double>("ellipse_max_ratio", ellipse_max_ratio_, 0.8);
      nh_.param<double>("hole_radius", hole_radius_, 0.0035);
      nh_.param<double>("holes_min_spacing", holes_min_spacing_, 1.0);
      nh_.param<std::string>("base_frame", base_frame_, "link_0");
      nh_.param<double>("max_dist", max_dist_, 1000);
      nh_.param<double>("max_angle_dist", max_angle_dist_, 1000);
      nh_.param<bool>("debug", debug_, true);

      // OpenCV windows 
      if(debug_){
        cv::namedWindow("Original Image");
        cv::namedWindow("1/MedianBlur");
        cv::namedWindow("2/MonoImage");
        cv::namedWindow("3/AdaptiveThresholding");
        cv::namedWindow("4/Opening");
        cv::namedWindow("Ellipse detection");
        cv::waitKey(10);
      }

      locking_data_ = false;
      tf_listener_ = new tf::TransformListener();
      img_sub_ = image_transport_.subscribeCamera(img_in_topic_, 1, &FindHolePoseService::imgCallback,this);
      
      // Starting server
      service_server_ = nh_.advertiseService("find_hole_pose", &FindHolePoseService::findHolePose,this);
      ROS_INFO("find_hole_pose service ready !");
    };

    void imgCallback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info){
      if (locking_data_)
        return;
      else{
        current_image_ = cv_bridge::toCvCopy(image_msg);
        current_orig_img_ = cv_bridge::toCvCopy(image_msg);
        current_cam_info_ = *cam_info;
        
        // Original image
        if(debug_)
          cv::imshow("Original Image", current_image_->image);
          cv::waitKey(10);
      }
    }
    
    bool findHolePose(lwr_peg_in_hole::FindHolePoseService::Request &req, lwr_peg_in_hole::FindHolePoseService::Response &res){
      // Stoping callback on image
      locking_data_ = true;
      
      /** Image processing to find the ellipses in the image **/
      // MedianBlur to help edge detection
      cv::medianBlur(current_image_->image, current_image_->image, 11);
      if(debug_)
        cv::imshow("1/MedianBlur", current_image_->image);
      
      // Grayscale image
      if(current_image_->image.channels() >1)
        cv::cvtColor(current_image_->image, current_image_->image, cv::COLOR_RGB2GRAY);
      if(debug_)
        cv::imshow("2/MonoImage", current_image_->image);
      
      // Adaptive threshold giving almost only edges
      cv::adaptiveThreshold(current_image_->image, current_image_->image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
      if(debug_)
        cv::imshow("3/AdaptiveThresholding", current_image_->image);
      
      // Opening to remove extra edges
      int morph_size = 1;
      cv::Mat morph_kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
      cv::morphologyEx(current_image_->image, current_image_->image, cv::MORPH_CLOSE, morph_kernel);
      if(debug_)
        cv::imshow("4/Opening", current_image_->image);
      
      cv::bitwise_not(current_image_->image, current_image_->image);
      
      // Find contours
      std::vector< std::vector<cv::Point> >  contours;
      cv::findContours( current_image_->image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );  
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
      
      if(final_ellipses.size() == 0){
        if (debug_)
          cv::waitKey(10);
        locking_data_ = false;
        return false;
      }
      
      /** Printing ellipses on the image **/
      if(debug_){
        cv::RNG rng(12345);
        for(int i = 0; i < final_ellipses.size() ; i++){
          cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          cv::ellipse(current_orig_img_->image,final_ellipses[i], color, 1, 8);
        }
        cv::imshow("Ellipse detection", current_orig_img_->image);
      }
      
      /** Convert the ellipse 2D to a circle 3D pose **/
      std::vector<geometry_msgs::PoseStamped> ellipses_pose1, ellipses_pose2;
      for(int i = 0; i< final_ellipses.size() ; i++){
        std::vector<geometry_msgs::Pose> holes_pose = ellipse_to_pose(final_ellipses[i], current_cam_info_, hole_radius_);
        if (holes_pose.size() > 0){
          geometry_msgs::PoseStamped pose_in, pose_converted;
          try{
            tf_listener_->waitForTransform(current_cam_info_.header.frame_id, base_frame_, ros::Time(0.0), ros::Duration(1.0));
            pose_in.header.frame_id = current_cam_info_.header.frame_id;
            pose_in.pose = holes_pose[0];
            tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
            
            ellipses_pose1.push_back(pose_converted);
            pose_in.pose = holes_pose[1];
            tf_listener_->transformPose(base_frame_, pose_in, pose_converted);
            ellipses_pose2.push_back(pose_converted);
          }
          catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            if (debug_)
              cv::waitKey(10);
            locking_data_ = false;
            return false;
          }
        }
      }
    
      /** Compare the positions to the estimation and keep the closest **/
      int closest_hole = -1;
      double closest_dist = -1.0, current_dist;
      tf::Transform tf_estimate, tf_computed, tf_diff;
      tf_estimate.setOrigin(tf::Vector3(req.hole_pose_estimate.position.x, req.hole_pose_estimate.position.y, req.hole_pose_estimate.position.z));
      tf_estimate.setRotation(tf::Quaternion(req.hole_pose_estimate.orientation.x, req.hole_pose_estimate.orientation.y, req.hole_pose_estimate.orientation.z, req.hole_pose_estimate.orientation.w));
      for(int i=0; i<ellipses_pose1.size(); i++){
        
        tf_computed.setOrigin(tf::Vector3(ellipses_pose1[i].pose.position.x, ellipses_pose1[i].pose.position.y, ellipses_pose1[i].pose.position.z));
        tf_computed.setRotation(tf::Quaternion(ellipses_pose1[i].pose.orientation.x, ellipses_pose1[i].pose.orientation.y, ellipses_pose1[i].pose.orientation.z, ellipses_pose1[i].pose.orientation.w));
        
        tf_diff = tf_estimate.inverseTimes(tf_computed);
        
        current_dist = sqrt( pow(tf_diff.getOrigin().getX(),2) + pow(tf_diff.getOrigin().getY(),2) + pow(tf_diff.getOrigin().getZ(),2) );
        
        if((current_dist < closest_dist) || (closest_dist < 0) ){
          closest_dist = current_dist;
          closest_hole = i;
        }
        
      }
      
      /** Check if closest is close enough **/
      if((closest_dist > max_dist_) || (closest_hole < 0)){
        if (debug_){
          ROS_ERROR("There is no holes close enough to the estimate");
          cv::waitKey(10);
        }
        locking_data_ = false;
        return false;
      }
      
      double roll, pitch, yaw, orientation_dist1, orientation_dist2, closest_orientation;
      
      tf_computed.setOrigin(tf::Vector3(ellipses_pose1[closest_hole].pose.position.x, ellipses_pose1[closest_hole].pose.position.y, ellipses_pose1[closest_hole].pose.position.z));
      tf_computed.setRotation(tf::Quaternion(ellipses_pose1[closest_hole].pose.orientation.x, ellipses_pose1[closest_hole].pose.orientation.y, ellipses_pose1[closest_hole].pose.orientation.z, ellipses_pose1[closest_hole].pose.orientation.w));        
      tf_diff = tf_estimate.inverseTimes(tf_computed);
      tf::Matrix3x3 m(tf_diff.getRotation());
      m.getRPY(roll, pitch, yaw);
      orientation_dist1 = sqrt( pow(roll,2) + pow(pitch,2) + pow(yaw,2) );

      tf_computed.setOrigin(tf::Vector3(ellipses_pose2[closest_hole].pose.position.x, ellipses_pose2[closest_hole].pose.position.y, ellipses_pose2[closest_hole].pose.position.z));
      tf_computed.setRotation(tf::Quaternion(ellipses_pose2[closest_hole].pose.orientation.x, ellipses_pose2[closest_hole].pose.orientation.y, ellipses_pose2[closest_hole].pose.orientation.z, ellipses_pose2[closest_hole].pose.orientation.w));        
      tf_diff = tf_estimate.inverseTimes(tf_computed);
      tf::Matrix3x3 m2(tf_diff.getRotation());
      m2.getRPY(roll, pitch, yaw);
      orientation_dist2 = sqrt( pow(roll,2) + pow(pitch,2) + pow(yaw,2) );
      
      if (std::min(orientation_dist1,orientation_dist2) < max_angle_dist_){
        
        if(orientation_dist1 < orientation_dist2){
          res.hole_pose.position.x = ellipses_pose1[closest_hole].pose.position.x;
          res.hole_pose.position.y = ellipses_pose1[closest_hole].pose.position.y;
          res.hole_pose.position.z = ellipses_pose1[closest_hole].pose.position.z;
          res.hole_pose.orientation.x = ellipses_pose1[closest_hole].pose.orientation.x;
          res.hole_pose.orientation.y = ellipses_pose1[closest_hole].pose.orientation.y;
          res.hole_pose.orientation.z = ellipses_pose1[closest_hole].pose.orientation.z;
          res.hole_pose.orientation.w = ellipses_pose1[closest_hole].pose.orientation.w;
        }else{
          res.hole_pose.position.x = ellipses_pose2[closest_hole].pose.position.x;
          res.hole_pose.position.y = ellipses_pose2[closest_hole].pose.position.y;
          res.hole_pose.position.z = ellipses_pose2[closest_hole].pose.position.z;
          res.hole_pose.orientation.x = ellipses_pose2[closest_hole].pose.orientation.x;
          res.hole_pose.orientation.y = ellipses_pose2[closest_hole].pose.orientation.y;
          res.hole_pose.orientation.z = ellipses_pose2[closest_hole].pose.orientation.z;
          res.hole_pose.orientation.w = ellipses_pose2[closest_hole].pose.orientation.w;
        }
        
      }else{
        if (debug_){
          ROS_ERROR("The closest hole has an orientation too far from the estimate");
          cv::waitKey(10);
        }
        locking_data_ = false;
        return false;
      }

      // Starting over callback on image
        locking_data_ = false;
        if (debug_)
          cv::waitKey(10);
        return true;
      
    }

    std::vector<geometry_msgs::Pose> ellipse_to_pose(cv::RotatedRect ellipse, sensor_msgs::CameraInfo camera_info, double radius){
      
    //    * Copyright (c) 2014 Markus Bader <markus.bader@tuwien.ac.at>
    //    * All rights reserved.
    //   Got from v4r_ellipses ROS package under BSD licence
    //   Method was coded following this paper:
    //   http://link.springer.com/chapter/10.1007/978-3-540-24672-5_41
      
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

    // Global variables
    ros::NodeHandle nh_, nh_priv;
    ros::ServiceServer service_server_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber img_sub_;
    cv_bridge::CvImagePtr current_image_, current_orig_img_;
    sensor_msgs::CameraInfo current_cam_info_;
    tf::TransformListener* tf_listener_;
    std::string img_in_topic_, base_frame_;
    int contour_min_pix_size_;
    double ellipse_max_ratio_, hole_radius_, holes_min_spacing_, max_dist_, max_angle_dist_;
    bool debug_, locking_data_;

};

#endif