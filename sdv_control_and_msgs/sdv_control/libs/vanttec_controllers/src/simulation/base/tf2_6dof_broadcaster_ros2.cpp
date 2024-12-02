/** ----------------------------------------------------------------------------
 * @file: tf2_6dof_broadcaster.cpp
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

#include "tf2_6dof_broadcaster_ros2.hpp"

TF2Broadcaster::TF2Broadcaster(rclcpp::Node::SharedPtr node,const std::string& _parent, const std::string& _child):
                 node_(node), parent_frame(_parent), child_frame(_child)
{
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

TF2Broadcaster::~TF2Broadcaster(){}

void TF2Broadcaster::broadcastTransform(const sdv_msgs::msg::EtaPose& _pose)
{        
    // To visualize correctly in RViz in the desired reference frame, just make sure to define correctly
    // your tfs and select the proper fixed frame (ex: map --> odom(ned) --> base_link)
    // The fixed frame would be map, and by displaying the tfs, viewing odom, measuremnts would make sense

    // MAKE SURE YOU KNOW YOUR FRAMES!!!
}