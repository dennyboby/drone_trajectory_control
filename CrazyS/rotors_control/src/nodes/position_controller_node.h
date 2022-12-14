/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2020 Ria Sonecha, MIT, USA
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CRAYZFLIE_2_POSITION_CONTROLLER_NODE_H
#define CRAYZFLIE_2_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>


#include "rotors_control/common.h"
#include "rotors_control/position_controller.h"
#include "rotors_control/mellinger_controller.h"
#include "rotors_control/internal_model_controller.h"
#include "rotors_control/crazyflie_complementary_filter.h"


namespace rotors_control {

    class PositionControllerNode{
        public:
            PositionControllerNode();
            ~PositionControllerNode();

            void InitializeParams();
            void Publish();

        private:

            bool waypointHasBeenPublished_ = false;
            bool enable_state_estimator_ = false;
            bool enable_mellinger_controller_ = false;
            bool enable_internal_model_controller_ = false;

            PositionController position_controller_;
            MellingerController mellinger_controller_;
            InternalModelController internal_model_controller_;
            sensorData_t sensors_;
            ros::Time imu_msg_head_stamp_;

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_Attitude_;
            ros::Timer timer_highLevelControl;
            ros::Timer timer_IMUUpdate;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitudeEstimation(const ros::TimerEvent& event);
            void CallbackHightLevelControl(const ros::TimerEvent& event);
            void CallbackIMUUpdate(const ros::TimerEvent& event);

            //subscribers
            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber cmd_multi_dof_joint_trajectory_spline_sub_;
            ros::Subscriber odometry_sub_;
            ros::Subscriber imu_sub_;
            ros::Subscriber imu_ideal_sub_;

            //publisher
            ros::Publisher motor_velocity_reference_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
            void MultiDofJointTrajectoryMellingerCallback(const mav_msgs::DroneState& drone_state_msg);

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void MellingerOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

            void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);
            void IMUMellingerCallback(const sensor_msgs::ImuConstPtr& imu_msg); //When the Mellinger's controller is on

    };
}

#endif // CRAZYFLIE_2_POSITION_CONTROLLER_NODE_H
