/** ----------------------------------------------------------------------------
 * @file: sdc1_broadcaster.hpp
 * @date: August 23, 2023
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief:  TF2 broadcaster class to publish transformations for the odom and
 *          base link frames of the SDC1.
 *          Frame Conventions:
 *              - ODOM:
                    - x (front)
                    - y (right)
                    - z (down)
 *              - BASE_LINK:
                    - x (front)
                    - y (right)
                    - z (down)
 * -----------------------------------------------------------------------------
 **/

#ifndef __SDC1_BROADCASTER_H__
#define __SDC1_BROADCASTER_H__

#include "rclcpp/rclcpp.hpp"

#include "simulation/base/tf2_6dof_broadcaster_ros2.hpp"

class SDC1Broadcaster : public TF2Broadcaster
{
    public:
        SDC1Broadcaster(rclcpp::Node::SharedPtr node);
        ~SDC1Broadcaster();

        void broadcastTransform(const sdv_msgs::msg::EtaPose& msg);
};

#endif