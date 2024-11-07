/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.cpp
 * @date: April 10, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated vehicle and correctly
 *         represent it in RViz.
 * https://github.com/ros-visualization/rviz/issues/597
 * -----------------------------------------------------------------------------
 **/

#include "simulation/tf2_6dof_broadcaster.hpp"

TF2Broadcaster::TF2Broadcaster(const std::string& _parent, const std::string& _child)
{
    this->parent_frame = _parent;
    this->child_frame = _child;
}

TF2Broadcaster::~TF2Broadcaster(){}

void TF2Broadcaster::BroadcastTransform(const vanttec_msgs::EtaPose& _pose)
{    
    geometry_msgs::TransformStamped transformStamped;
    
    // To visualize correctly in RViz in the desired reference frame, just make sure to define correctly
    // your tfs and select the proper fixed frame (ex: map --> odom(ned) --> etc)
    // The fixed frame would be map, and by displaying the tfs, viewing odom, measuremnts would make sense
    transformStamped.header.stamp               = ros::Time::now();
    transformStamped.header.frame_id            = this->parent_frame;
    transformStamped.child_frame_id             = this->child_frame;
    transformStamped.transform.translation.x    = _pose.x;
    transformStamped.transform.translation.y    = _pose.y;
    transformStamped.transform.translation.z    = _pose.z;

    tf2::Quaternion q;
    q.setRPY(_pose.phi, _pose.theta, _pose.psi);
    q.normalize();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    this->br.sendTransform(transformStamped);

    geometry_msgs::PoseStamped      pose;

    pose.header.stamp       = ros::Time::now();
    pose.header.frame_id    = this->parent_frame;
    pose.pose.position.x    = _pose.x;
    pose.pose.position.y    = _pose.y;
    pose.pose.position.z    = _pose.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    this->path.header.stamp     = ros::Time::now();
    this->path.header.frame_id  = this->parent_frame;
    this->path.poses.push_back(pose);
}