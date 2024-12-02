/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.hpp
 * @date: April 10, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated vehicle and correctly
 *         represent it in RViz.
 * -----------------------------------------------------------------------------
 **/

#ifndef __TF2_6DOF_BROADCASTER_H__
#define __TF2_6DOF_BROADCASTER_H__

#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "vanttec_msgs/EtaPose.h"

class TF2Broadcaster
{
    public:
        
        nav_msgs::Path                  path;
        tf2_ros::TransformBroadcaster   br;

        std::string parent_frame;
        std::string child_frame;
        
        TF2Broadcaster(const std::string& _parent, const std::string& _child);
        ~TF2Broadcaster();

        void BroadcastTransform(const vanttec_msgs::EtaPose& msg);
};

#endif