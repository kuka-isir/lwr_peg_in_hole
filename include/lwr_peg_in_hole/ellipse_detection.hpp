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

#ifndef ELLIPSE_DETECTION_HPP
#define ELLIPSE_DETECTION_HPP

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

// Functions
void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info);

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
ros::Publisher pose_pub_;
tf::TransformListener* tf_listener_;
std::string img_in_topic_, base_frame_, out_pose_topic_;
int contour_min_pix_size_;
double ellipse_max_ratio_, hole_radius_, holes_min_spacing_;
bool debug_;

#endif