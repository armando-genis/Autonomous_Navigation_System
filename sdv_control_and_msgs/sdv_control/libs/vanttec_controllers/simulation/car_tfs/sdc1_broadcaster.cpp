/** ----------------------------------------------------------------------------
 * @file: sdc1_broadcaster.hpp
 * @date: August 23, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief:  TF2 broadcaster class to publish transformations for the odom and
 *          base link frames of the SDC1.
 *          Frame Conventions:
 *              - ODOM: (NED)
                    - x (front)
                    - y (right)
                    - z (down)
 *              - BASE_LINK:
                    - x (front)
                    - y (right)
                    - z (down)
 * -----------------------------------------------------------------------------
 **/
#include "sdc1_broadcaster.hpp"

SDC1Broadcaster::SDC1Broadcaster(rclcpp::Node::SharedPtr node):TF2Broadcaster(node, "odom", "base_link")
{}

SDC1Broadcaster::~SDC1Broadcaster(){}

void SDC1Broadcaster::broadcastTransform(const sdv_msgs::msg::EtaPose& _pose)
{    
    // The definition of this method considers the same coordinate frame convention 
    // for both the parent and child frame
    
    // To visualize correctly in RViz in the desired reference frame, just make sure to define correctly
    // your tfs and select the proper fixed frame (ex: map --> odom(ned) --> base_link)
    // The fixed frame would be map, and by displaying the tfs, viewing odom, measuremnts would make sense

    tf_stmpd_.header.stamp               = node_->now();
    tf_stmpd_.header.frame_id            = this->parent_frame;
    tf_stmpd_.child_frame_id             = this->child_frame;
    tf_stmpd_.transform.translation.x    = _pose.x;
    tf_stmpd_.transform.translation.y    = _pose.y;
    tf_stmpd_.transform.translation.z    = _pose.z;

    tf2::Quaternion q;
    q.setRPY(_pose.phi, _pose.theta, _pose.psi);
    q.normalize();
    tf_stmpd_.transform.rotation.x = q.x();
    tf_stmpd_.transform.rotation.y = q.y();
    tf_stmpd_.transform.rotation.z = q.z();
    tf_stmpd_.transform.rotation.w = q.w();

    this->br_->sendTransform(tf_stmpd_);

    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp       = node_->now();
    pose.header.frame_id    = this->parent_frame;
    pose.pose.position.x    = _pose.x;
    pose.pose.position.y    = _pose.y;
    pose.pose.position.z    = _pose.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    this->path_.header.stamp     = node_->now();
    this->path_.header.frame_id  = this->parent_frame;
    this->path_.poses.push_back(pose);
}