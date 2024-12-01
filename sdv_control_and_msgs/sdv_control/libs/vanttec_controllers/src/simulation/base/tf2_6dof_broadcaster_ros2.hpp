/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.hpp
 * @date: August 10, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief:  Base TF2 broadcaster class to publish transformations
 *          from a parent frame to a child frame.
 *          The broadcastTransform() method must be defined in derived classes
 *           as the transform is strictly dependent of the frame conventions used.
 * -----------------------------------------------------------------------------
 **/

#ifndef __TF2_6DOF_BROADCASTER2_H__
#define __TF2_6DOF_BROADCASTER2_H__

#include <math.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sdv_msgs/msg/eta_pose.hpp"

class TF2Broadcaster
{
    public:
        nav_msgs::msg::Path path_;

        /* Class methods */
        // The broadcastTransform() method must be defined in derived classes
        // as the transform is strictly dependent of the frame conventions used.
        virtual void broadcastTransform(const sdv_msgs::msg::EtaPose& msg);

    protected:
        rclcpp::Node::SharedPtr node_;
        std::string parent_frame;
        std::string child_frame;

        geometry_msgs::msg::TransformStamped tf_stmpd_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

        TF2Broadcaster(rclcpp::Node::SharedPtr node,const std::string& _parent, const std::string& _child);
        ~TF2Broadcaster();

};

#endif