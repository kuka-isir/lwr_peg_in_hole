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

#ifndef ESTIMATE_HOLES_SERVICE_HPP
#define ESTIMATE_HOLES_SERVICE_HPP

#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include <lwr_peg_in_hole/EstimateHolesService.h>

#define M_PI 3.14159265358979323846  /* pi */

class EstimateHolesService
{
  public:
    // Constructor.
    EstimateHolesService(){
      ros::WallDuration sleep_t(0.5);
      service_server_ = nh_.advertiseService("estimate_holes", &EstimateHolesService::estimate_holes_poses,this);
      ROS_INFO("Estimate_holes service ready !");
      base_frame_ = "link_0";
      tf_.reset(new tf::TransformListener(ros::Duration(1.0)));
      planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_));
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
    };

    void getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene){
      planning_scene_monitor_->requestPlanningSceneState();
      full_planning_scene = planning_scene_monitor_->getPlanningScene();
      full_planning_scene->getPlanningSceneMsg(planning_scene);
    }
    
    bool estimate_holes_poses(lwr_peg_in_hole::EstimateHolesService::Request &req, lwr_peg_in_hole::EstimateHolesService::Response &res){

      std::string object_name = req.object_name;

      // Check if holes poses are defined in YAML
      ROS_INFO_STREAM("Loading relative holes locations for "<<object_name);
      YAML::Node holes_config = YAML::LoadFile(ros::package::getPath("lwr_peg_in_hole")+"/holes/"+object_name+".yaml");
      if (!holes_config["holes"]) {
        ROS_ERROR("Couldn't read holes location");
        return false;
      }
      YAML::Node poses = holes_config["holes"];  
      holes_location_.clear();
      for(unsigned int i =0; i<poses.size(); i++){
        std::vector<float> pose = poses[i].as<std::vector<float> >();
        holes_location_.push_back(pose);
      }
      
      // Check if object is in scene
      getPlanningScene(planning_scene_msg_, full_planning_scene_);
      moveit_msgs::CollisionObjectPtr coll_obj;
      for(int i=0;i<planning_scene_msg_.world.collision_objects.size();i++){
        if(planning_scene_msg_.world.collision_objects[i].id == object_name){ 
          coll_obj.reset(new moveit_msgs::CollisionObject(planning_scene_msg_.world.collision_objects[i]));
        }
      }
      if(coll_obj == NULL){
        ROS_ERROR_STREAM("Failed to find object "<< object_name<< " in the scene !!!");
        return false;
      }
      
      // Get pose of object in scene
      geometry_msgs::PoseStamped obj_pose;
      obj_pose.header = coll_obj->header;
      obj_pose.pose = coll_obj->mesh_poses[0];
      tf_->transformPose(base_frame_, obj_pose, obj_pose);
      
      tf::Transform object_transform;
      object_transform.setOrigin(tf::Vector3(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z));
      object_transform.setRotation(tf::Quaternion(obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z, obj_pose.pose.orientation.w));
      
      // Transform holes poses to link_0
      std::vector<geometry_msgs::Pose> final_poses;
      for(int hole_nb = 0; hole_nb < poses.size(); hole_nb++){
        tf::Transform hole_transform;
        hole_transform.setOrigin(tf::Vector3(holes_location_[hole_nb][0], holes_location_[hole_nb][1], holes_location_[hole_nb][2]));
        tf::Quaternion rotation_to_hole;
        rotation_to_hole.setRPY(holes_location_[hole_nb][3],holes_location_[hole_nb][4],holes_location_[hole_nb][5]);
        hole_transform.setRotation(rotation_to_hole);
        
        tf::Transform final_transform = object_transform*hole_transform;
        geometry_msgs::Pose final_pose;
        final_pose.position.x = final_transform.getOrigin().getX();
        final_pose.position.y = final_transform.getOrigin().getY();
        final_pose.position.z = final_transform.getOrigin().getZ();
        final_pose.orientation.x = final_transform.getRotation().getX();
        final_pose.orientation.y = final_transform.getRotation().getY();
        final_pose.orientation.z = final_transform.getRotation().getZ();
        final_pose.orientation.w = final_transform.getRotation().getW();
        final_poses.push_back(final_pose);
      }
      
      // Return poses
      res.holes_poses = final_poses;
      return true;
    };

    ros::NodeHandle nh_;
    ros::ServiceServer service_server_;
    boost::scoped_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    planning_scene::PlanningScenePtr full_planning_scene_;
    boost::shared_ptr<tf::TransformListener> tf_;
    std::vector<std::vector<float> > holes_location_;
    std::string base_frame_;

};


#endif
