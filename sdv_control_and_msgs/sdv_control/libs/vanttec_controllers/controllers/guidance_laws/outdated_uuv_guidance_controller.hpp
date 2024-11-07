/** ----------------------------------------------------------------------------
 * @file: uuv_guidance_controller.hpp
 * @date: July 30, 2020
 * @modified: June 20, 2021
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Guidance controller, which manages the different guidance laws 
 *         available to the UUV.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_GUIDANCE_CONTROLLER_H__
#define __UUV_GUIDANCE_CONTROLLER_H__

#include <cmath>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <vanttec_msgs/GuidanceWaypoints.h>
#include <vanttec_msgs/MasterStatus.h>

#include "asmc_guidance_4dof.hpp"

/********** Helper Constants ***********/

// const float     PI                      = 3.14159;
const uint8_t   LOS_WAYPOINT_OFFSET     = 2;

/********** Guidance Laws ***********/

typedef enum GuidanceLaws_E
{
    NONE = 0,
    LOS_GUIDANCE_LAW = 1,
    ORBIT_GUIDANCE_LAW = 2,
    ASMC_GUIDANCE_LAW = 3
} GuidanceLaws_E;


/***************** 2D LOS ******************/

/* Enum for the 2D LOS Guidance Law States */

typedef enum LOSLawStates_E
{
    LOS_LAW_STANDBY = 0,
    LOS_LAW_DEPTH_NAV = 1,
    LOS_LAW_WAYPOINT_NAV = 2,
} LOSLawStates_E;

/* 2D LOS Guidance Law Struct */

typedef struct LOSLawStateMachine_S
{
    LOSLawStates_E      state_machine;
    int                 current_waypoint;      
} LOSLawStateMachine_S;

/***************** Orbit ******************/

typedef enum OrbitLawStates_E
{
    ORBIT_LAW_STANDBY = 0,
    ORBIT_LAW_DEPTH_NAV = 1,
    ORBIT_LAW_WAYPOINT_NAV = 2,
} OrbitLawStates_E;

/* Orbit Guidance Law Struct */

typedef struct OrbitLawStateMachine_S
{
    OrbitLawStates_E    state_machine;
    int                 current_waypoint;      
} OrbitLawStateMachine_S;

/***************** ASMC_ ******************/

typedef enum ASMCLawStates_E
{
    ASMC_LAW_STANDBY = 0,
    ASMC_LAW_WAYPOINT_NAV = 1,
} ASMCLawStates_E;


/* ASMC Guidance Law Struct */

typedef struct ASMCLawStateMachine_S
{
    ASMCLawStates_E    state_machine;
    int                current_waypoint;      
} ASMCLawStateMachine_S;


/********** Guidance Controller ***********/

class GuidanceController
{
    public:
        GuidanceLaws_E          current_guidance_law;
        LOSLawStateMachine_S    los_state_machine;
        OrbitLawStateMachine_S  orbit_state_machine;
        ASMCLawStateMachine_S   asmc_state_machine;
        
        geometry_msgs::Pose                 current_positions_ned;
        geometry_msgs::Twist                desired_set_points;
        vanttec_msgs::GuidanceWaypoints      current_waypoint_list;
        vanttec_msgs::MasterStatus           uuv_status;

        GuidanceController(const float SAMPLE_TIME_S);
        ~GuidanceController();
        
        void OnCurrentPositionReception(const geometry_msgs::Pose& _pose);
        void OnWaypointReception(const vanttec_msgs::GuidanceWaypoints& _waypoints);
        void OnEmergencyStop(const std_msgs::Empty& _msg);
        void OnMasterStatus(const vanttec_msgs::MasterStatus& _status);
        void updateStateMachines();

    private:
        ASMC_GUIDANCE_4DOF ASMC_Guidance;
        
        /* ASMC Parameters */        
        float asmc_guidance_position_error_threshold;
        float asmc_guidance_radial_error_threshold;
        float asmc_guidance_speed_gain;
        float asmc_guidance_euclidean_distance;
        float asmc_guidance_radial_distance;

        /* LOS Parameters */        
        float los_depth_error_threshold;
        float los_position_error_threshold;
        float los_lookahead_distance;
        float los_max_speed;
        float los_min_speed;
        float los_speed_gain;
        float los_euclidean_distance;

        /* Orbit Parameters */
        float orbit_depth_error_threshold;
        float orbit_position_error_threshold;
        float orbit_lookahead_distance;
        float orbit_max_speed;
        float orbit_min_speed;
        float orbit_speed_gain;
        float orbit_euclidean_distance;

};

#endif