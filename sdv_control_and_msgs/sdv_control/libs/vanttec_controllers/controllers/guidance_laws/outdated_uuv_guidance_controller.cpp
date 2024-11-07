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

#include <uuv_guidance_controller.hpp>

GuidanceController::GuidanceController(const float SAMPLE_TIME_S) : ASMC_Guidance((double) SAMPLE_TIME_S, 0.001, 0.01, 0.01, 0.01, 0.1)
{
    /* Desired speed output initalization */
    this->desired_set_points.linear.x = 0;
    this->desired_set_points.linear.y = 0;
    this->desired_set_points.linear.z = 0;
    this->desired_set_points.angular.z = 0;

    /* State Machines Initialization */
    this->current_guidance_law = NONE;
    this->los_state_machine.state_machine = LOS_LAW_STANDBY;
    this->asmc_state_machine.state_machine = ASMC_LAW_STANDBY;

    /* ASMC Parameter Init */
    this->asmc_state_machine.current_waypoint = 0;
    this->asmc_guidance_position_error_thresholD_= 0.1;
    this->asmc_guidance_radial_error_thresholD_= 0.02;
    this->asmc_guidance_euclidean_distance = 0;
    this->asmc_guidance_radial_distance = 0;
    this->asmc_guidance_speed_gain = 100;

    /* LOS Parameter Init */
    this->los_state_machine.current_waypoint = 0;
    this->los_depth_error_thresholD_= 0.01;
    this->los_lookahead_distance = 0.9; // Lookahead distance corresponds to 2 times the length of the UUV
    this->los_min_speeD_= 0;
    this->los_max_speeD_= 0.9;
    this->los_position_error_thresholD_= 0.4;
    this->los_euclidean_distance = 0;
    this->los_speed_gain = 100;

    /* Orbit Parameter Init */
    this->orbit_state_machine.current_waypoint = 0;
    this->orbit_depth_error_thresholD_= 0.01;
    this->orbit_lookahead_distance = 0.9; // Lookahead distance corresponds to 2 times the length of the UUV
    this->orbit_min_speeD_= 0;
    this->orbit_max_speeD_= 0.9;
    this->orbit_position_error_thresholD_= 0.4;
    this->orbit_euclidean_distance = 0;
    this->orbit_speed_gain = 100;

    ASMC_Guidance.asmc_guidance_yaw.Kalpha = 0.1;
}

GuidanceController::~GuidanceController(){}

void GuidanceController::OnCurrentPositionReception(const geometry_msgs::Pose& _pose)
{
    /* Store the current position in NED coordinates */
    this->current_positions_ned.position.x      = _pose.position.x;
    this->current_positions_ned.position.y      = _pose.position.y;
    this->current_positions_ned.position.z      = _pose.position.z;

    // double w = _pose.orientation.w;
    // double x = _pose.orientation.x;
    // double y = _pose.orientation.y;
    // double z = _pose.orientation.z;
    // double C_11 = pow(w,2)+pow(x,2)-pow(y,2)-pow(z,2);
    // double C_12 = 2*(x*y + w*z);
    // double C_13 = 2*(x*z - w*y);
    // double C_23 = 2*(y*z + w*x);
    // double C_33 = pow(w,2)-pow(x,2)-pow(y,2)+pow(z,2);
    // //3-2-1 convention
    // double yaw = atan2(C_12,C_11);
    // double pitch = -asin(C_13);
    // double roll = atan2(C_23,C_33);

    // this->current_positions_ned.orientation.z   = yaw;
    this->current_positions_ned.orientation.z   = _pose.orientation.z;
}

void GuidanceController::OnWaypointReception(const vanttec_msgs::GuidanceWaypoints& _waypoints)
{
    /* Waypoints update (and therefore, guidance law triggering) can only be done when the guidance
    node is not executing any other type of action/law; only acceptable input is an emergency stop */

    /* Trigger the appropriate guidance law state machine */
    switch((GuidanceLaws_E)_waypoints.guidance_law)
    {
        case ASMC_GUIDANCE_LAW:
            this->asmc_state_machine.state_machine = ASMC_LAW_WAYPOINT_NAV;
            this->los_state_machine.state_machine = LOS_LAW_STANDBY;
            this->orbit_state_machine.state_machine = ORBIT_LAW_STANDBY;
            break;
        case LOS_GUIDANCE_LAW:
            this->los_state_machine.state_machine = LOS_LAW_DEPTH_NAV;
            this->orbit_state_machine.state_machine = ORBIT_LAW_STANDBY;
            this->asmc_state_machine.state_machine = ASMC_LAW_STANDBY;
            break;
        case ORBIT_GUIDANCE_LAW:
            this->orbit_state_machine.state_machine = ORBIT_LAW_DEPTH_NAV;
            this->los_state_machine.state_machine = LOS_LAW_STANDBY;
            this->asmc_state_machine.state_machine = ASMC_LAW_STANDBY;
            break;
        case NONE:
            this->desired_set_points.linear.x = _waypoints.waypoint_list_x[0];
            this->desired_set_points.linear.y = _waypoints.waypoint_list_y[0];
            this->desired_set_points.linear.z = _waypoints.depth_set_point;
            this->desired_set_points.angular.z = _waypoints.heading_set_point;
            break;
        default:
            break;
    }

    this->los_state_machine.current_waypoint = 0;
    this->orbit_state_machine.current_waypoint = 0;
    this->asmc_state_machine.current_waypoint = 0;

    /* update the current guidance law selection and the internal waypoint list */
    this->current_guidance_law = (GuidanceLaws_E) _waypoints.guidance_law;
    this->current_waypoint_list = _waypoints;

}

void GuidanceController::OnEmergencyStop(const std_msgs::Empty& _msg)
{
    /* Stop the vehicle from moving. Depth and heading keep the previous setpoint. */
    this->desired_set_points.linear.x = 0;
    this->desired_set_points.linear.y = 0;

    /* reset the guidance law and the state machines */
    this->current_guidance_law = NONE;
    this->los_state_machine.state_machine = LOS_LAW_STANDBY;
    this->asmc_state_machine.state_machine = ASMC_LAW_STANDBY;
}

void GuidanceController::OnMasterStatus(const vanttec_msgs::MasterStatus& _status)
{
    /* Store the current status */
    this->uuv_status = _status;
}


void GuidanceController::updateStateMachines()
{
    /* Enter a specific state machine according to the selected guidance law. */
    switch(this->current_guidance_law)
    {
        case ASMC_GUIDANCE_LAW:
            switch(this->asmc_state_machine.state_machine)
            {
                case ASMC_LAW_STANDBY:
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = 0;
                    this->asmc_state_machine.current_waypoint = 0;
                    break;
                case ASMC_LAW_WAYPOINT_NAV:
                    float waypoint_x = this->current_waypoint_list.waypoint_list_x[this->asmc_state_machine.current_waypoint];
                    float uuv_x = this->current_positions_ned.position.x;
                    float waypoint_y = this->current_waypoint_list.waypoint_list_y[this->asmc_state_machine.current_waypoint];
                    float uuv_y = this->current_positions_ned.position.y;
                    float waypoint_z = this->current_waypoint_list.waypoint_list_y[this->asmc_state_machine.current_waypoint];
                    float uuv_z = this->current_positions_ned.position.z;
                    float set_points[4];

                    set_points[0] = waypoint_x;
                    set_points[1] = waypoint_y;
                    set_points[2] = waypoint_z;
                    set_points[3] = std::atan2((waypoint_y - uuv_y),(waypoint_x - uuv_x)); //this->current_waypoint_list.heading_set_point;//

                    std::cout << set_points[0] << std::endl;
                    std::cout << set_points[1] << std::endl;
                    std::cout << set_points[2] << std::endl;
                    std::cout << set_points[3] << std::endl;

                    ASMC_Guidance.SetSetpoints(set_points);
                    ASMC_Guidance.calculateManipulation(current_positions_ned);

                    this->desired_set_points.linear.x = ASMC_Guidance.U(0);
                    this->desired_set_points.linear.y = ASMC_Guidance.U(1);
                    this->desired_set_points.linear.z = ASMC_Guidance.U(2);
                    this->desired_set_points.angular.z = ASMC_Guidance.U(3);

                    this->asmc_guidance_euclidean_distance = std::sqrt(std::pow((waypoint_x - uuv_x), 2)
                                                                 + std::pow((waypoint_y - uuv_y), 2)
                                                                 + std::pow((waypoint_z - uuv_z), 2));


                    if (this->asmc_guidance_euclidean_distance <= this->asmc_guidance_position_error_threshold)
                    {
                        this->desired_set_points.linear.x = 0;
                        this->desired_set_points.linear.y = 0;
                        this->desired_set_points.linear.z = 0;
                        this->asmc_guidance_euclidean_distance = 0;

                        if (this->asmc_guidance_radial_distance <= this->asmc_guidance_radial_error_threshold)
                        {
                            this->desired_set_points.angular.z = 0;
                            this->asmc_guidance_radial_distance = 0;

                            if ((this->asmc_state_machine.current_waypoint + 1) < this->current_waypoint_list.waypoint_list_length)
                            {
                                this->asmc_state_machine.current_waypoint ++;
                            }
                            else
                            {
                                this->asmc_state_machine.state_machine = ASMC_LAW_STANDBY;
                                this->asmc_state_machine.current_waypoint = 0;
                                this->current_guidance_law = NONE;
                            }
                        }
                    }
                    break;
            }
            break;

        /* Line-Of-Sight Guidance Law
           Strategy:
                - Navigate to the specified waypoint depth so that 2D LOS can be used.
                - Compute the desired speed and heading according to the LOS algorithM_.
                - When we are in the vicinity of the target waypoint (i.e. euclidean distance < threshold), stop.
                - If there are more waypoints, continue with the next. If not, return to standby.
        */

        case LOS_GUIDANCE_LAW:

            switch(this->los_state_machine.state_machine)
            {
                case LOS_LAW_STANDBY:
                {
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = 0;
                    this->los_state_machine.current_waypoint = 0;
                    break;
                }
                case LOS_LAW_DEPTH_NAV:
                {
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = 0;
                    //this->desired_set_points.angular.z = 0;
                    this->desired_set_points.linear.z = this->current_waypoint_list.waypoint_list_z[this->los_state_machine.current_waypoint + 1];
                    float los_depth_error = std::abs((float) this->current_positions_ned.position.z - (float) this->desired_set_points.linear.z);
                    if (los_depth_error <= this->los_depth_error_threshold)
                    {
                        this->los_state_machine.state_machine = LOS_LAW_WAYPOINT_NAV;
                    }
                    break;
                }
                case LOS_LAW_WAYPOINT_NAV:
                {
                    /* Create references for readability */
                    float x_k  = this->current_waypoint_list.waypoint_list_x[this->los_state_machine.current_waypoint];
                    float x_k1 = this->current_waypoint_list.waypoint_list_x[this->los_state_machine.current_waypoint + 1];

                    float y_k  = this->current_waypoint_list.waypoint_list_y[this->los_state_machine.current_waypoint];
                    float y_k1 = this->current_waypoint_list.waypoint_list_y[this->los_state_machine.current_waypoint + 1];

                    float x_uuv = this->current_positions_ned.position.x;
                    float y_uuv = this->current_positions_ned.position.y;

                    /* Algorithm */
                    float alpha_k = std::atan2((y_k1 - y_k), (x_k1 - x_k));

                    /*
                    if (std::abs(alpha_k) > PI)
                    {
                        alpha_k = (alpha_k/std::abs(alpha_k)) * (std::abs(alpha_k) - 2 * PI);
                    }
                    */

                    float along_track_distance = (x_uuv - x_k) * std::cos(alpha_k) + (y_uuv - y_k) * std::sin(alpha_k);
                    float cross_track_error = - (x_uuv - x_k) * std::sin(alpha_k) + (y_uuv - y_k) * std::cos(alpha_k);

                    float total_distance = (x_k1 - x_k) * std::cos(alpha_k) + (y_k1 - y_k) * std::sin(alpha_k);

                    if (along_track_distance > total_distance)
                    {
                        alpha_k = alpha_k - PI;
                        if (std::abs(alpha_k) > PI)
                        {
                            alpha_k = (alpha_k/std::abs(alpha_k)) * (std::abs(alpha_k) - 2 * PI);
                        }
                        along_track_distance = (x_uuv - x_k) * std::cos(alpha_k) + (y_uuv - y_k) * std::sin(alpha_k);
                        cross_track_error = - (x_uuv - x_k) * std::sin(alpha_k) + (y_uuv - y_k) * std::cos(alpha_k);
                    }

                    float desired_heading_ = alpha_k + std::atan(-(cross_track_error/this->los_lookahead_distance));

                    /*if (abs(desired_heading) > PI)
                    {
                        desired_heading_ = (desired_heading/abs(desired_heading)) * (abs(desired_heading) - 2 * PI);
                    }*/

                    float desired_velocity = (this->los_max_speed - this->los_min_speed) *
                                             (1 - (std::abs(along_track_distance) / std::sqrt(std::pow(along_track_distance, 2) + this->los_speed_gain)));

                    if (std::abs(this->current_positions_ned.orientation.z - desired_heading) > 0.75)
                    {
                        float desired_velocity = 0.075;
                    }

                    this->desired_set_points.linear.x = desired_velocity;
                    this->desired_set_points.linear.y = 0;
                    this->desired_set_points.angular.z = desired_heading;

                    this->los_euclidean_distance = std::sqrt(std::pow((x_k1 - x_uuv), 2) + std::pow((y_k1 - y_uuv), 2));

                    if (this->los_euclidean_distance <= this->los_position_error_threshold)
                    {
                        this->desired_set_points.linear.x = 0;
                        this->desired_set_points.linear.y = 0;
                        this->los_euclidean_distance = 0;

                        if ((this->los_state_machine.current_waypoint + LOS_WAYPOINT_OFFSET) < this->current_waypoint_list.waypoint_list_length)
                        {
                            this->los_state_machine.current_waypoint += 1;
                            this->los_state_machine.state_machine = LOS_LAW_DEPTH_NAV;
                        }
                        else
                        {
                            this->los_state_machine.state_machine = LOS_LAW_STANDBY;
                            this->los_state_machine.current_waypoint = 0;
                            this->current_guidance_law = NONE;
                        }
                    }
                    break;
                }
            }
            break;

        /* Orbit Guidance Law
           Strategy:
                - Compute the desired speed and heading according to the LOS algorithm, adding 90 degrees to heading
                  and restricting movement in surge.
                - When we are in the vicinity of the target waypoint (i.e. euclidean distance < threshold), stop.
                - If there are more waypoints, continue with the next. If not, return to standby.
        */

        case ORBIT_GUIDANCE_LAW:

            switch(this->orbit_state_machine.state_machine)
            {
                case ORBIT_LAW_STANDBY:
                {
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = 0;
                    this->orbit_state_machine.current_waypoint = 0;
                    break;
                }

                case ORBIT_LAW_DEPTH_NAV:
                {
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = 0;
                    //this->desired_set_points.angular.z = 0;
                    this->desired_set_points.linear.z = this->current_waypoint_list.waypoint_list_z[this->orbit_state_machine.current_waypoint + 1];
                    float orbit_depth_error = std::abs((float) this->current_positions_ned.position.z - (float) this->desired_set_points.linear.z);
                    if (orbit_depth_error <= this->orbit_depth_error_threshold)
                    {
                        this->orbit_state_machine.state_machine = ORBIT_LAW_WAYPOINT_NAV;
                    }
                    break;
                }

                case ORBIT_LAW_WAYPOINT_NAV:
                {
                    /* Create references for readability */
                    float x_k  = this->current_waypoint_list.waypoint_list_x[this->orbit_state_machine.current_waypoint];
                    float x_k1 = this->current_waypoint_list.waypoint_list_x[this->orbit_state_machine.current_waypoint + 1];

                    float y_k  = this->current_waypoint_list.waypoint_list_y[this->orbit_state_machine.current_waypoint];
                    float y_k1 = this->current_waypoint_list.waypoint_list_y[this->orbit_state_machine.current_waypoint + 1];

                    float x_uuv = this->current_positions_ned.position.x;
                    float y_uuv = this->current_positions_ned.position.y;

                    /* Algorithm */
                    float alpha_k = std::atan2((y_k1 - y_k), (x_k1 - x_k));

                    float along_track_distance = (x_uuv - x_k) * std::cos(alpha_k) + (y_uuv - y_k) * std::sin(alpha_k);
                    float cross_track_error = - (x_uuv - x_k) * std::sin(alpha_k) + (y_uuv - y_k) * std::cos(alpha_k);

                    float total_distance = (x_k1 - x_k) * std::cos(alpha_k) + (y_k1 - y_k) * std::sin(alpha_k);

                    if (along_track_distance > total_distance)
                    {
                        alpha_k = alpha_k - PI;
                        if (std::abs(alpha_k) > PI)
                        {
                            alpha_k = (alpha_k/std::abs(alpha_k)) * (std::abs(alpha_k) - 2 * PI);
                        }
                        along_track_distance = (x_uuv - x_k) * std::cos(alpha_k) + (y_uuv - y_k) * std::sin(alpha_k);
                        cross_track_error = - (x_uuv - x_k) * std::sin(alpha_k) + (y_uuv - y_k) * std::cos(alpha_k);
                    }

                    float desired_heading_ = alpha_k + std::atan(-(cross_track_error/this->los_lookahead_distance));

                    float desired_velocity = (this->orbit_max_speed - this->orbit_min_speed) *
                                             (1 - (std::abs(along_track_distance) / std::sqrt(std::pow(along_track_distance, 2) + this->orbit_speed_gain)));

                    if (std::abs(this->current_positions_ned.orientation.z - desired_heading) > 0.75)
                    {
                        float desired_velocity = 0.075;
                    }

                    //Change direction of movement
                    this->desired_set_points.linear.x = 0;
                    this->desired_set_points.linear.y = desired_velocity;
                    this->desired_set_points.angular.z = desired_heading - PI/2;
                    //this->desired_set_points.linear.x = 0;
                    //this->desired_set_points.linear.y = -desired_velocity;
                    //this->desired_set_points.angular.z = desired_heading + PI / 2;

                    this->orbit_euclidean_distance = std::sqrt(std::pow((x_k1 - x_uuv), 2) + std::pow((y_k1 - y_uuv), 2));

                    if (this->orbit_euclidean_distance <= this->orbit_position_error_threshold)
                    {
                        this->desired_set_points.linear.x = 0;
                        this->desired_set_points.linear.y = 0;
                        this->orbit_euclidean_distance = 0;

                        if ((this->orbit_state_machine.current_waypoint + LOS_WAYPOINT_OFFSET) < this->current_waypoint_list.waypoint_list_length)
                        {
                            this->orbit_state_machine.current_waypoint += 1;
                            this->orbit_state_machine.state_machine = ORBIT_LAW_DEPTH_NAV;
                        }
                        else
                        {
                            this->orbit_state_machine.state_machine = ORBIT_LAW_STANDBY;
                            this->orbit_state_machine.current_waypoint = 0;
                            this->current_guidance_law = NONE;
                        }
                    }
                    break;
                }
            }
            break;
        /* If no guidance law is being executed, keep desired speeds at 0 and maintain current depth and heading. */
        case NONE:
        default:
            break;
    }
}
