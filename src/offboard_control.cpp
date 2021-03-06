/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/navigator_mission_item.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node 
{
	public:
		explicit OffboardControl() : Node("offboard_control") 
		{
			offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
			position_setpoint_triplet_publisher_ =
				this->create_publisher<PositionSetpointTriplet>("PositionSetpointTriplet_PubSubTopic", 10);
			//mission_item_publisher_ =
		    //this->create_publisher<px4_msgs::msg::NavigatorMissionItem>("NavigatorMissionItem_PubSubTopic",10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
			
			// get common timestamp
			timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic",
				10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

			offboard_setpoint_counter_ = 0;

			auto timer_callback = [this]() -> void {
				if (offboard_setpoint_counter_ == 100) {
					// Change to Offboard mode after 100 setpoints
					this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

					// Arm the vehicle
					this->arm();
				}

				// offboard_control_mode needs to be paired with position_setpoint_triplet
				//publish_offboard_control_mode();
				//publish_position_setpoint_triplet();
				uint32_t command =16; 
				this->publish_vehicle_command(command,0.0,0.0);
				//this->publish_mission_item();

				// stop the counter after reaching 100
				if (offboard_setpoint_counter_ < 101) {
					offboard_setpoint_counter_++;
				}
			};
			timer_ = this->create_wall_timer(1s, timer_callback);
		}

		void arm() const;
		void disarm() const;

	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr position_setpoint_triplet_publisher_;
		rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Publisher<px4_msgs::msg::NavigatorMissionItem>::SharedPtr mission_item_publisher_;

		rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

		std::atomic<unsigned long long> timestamp_;   //!< common synced timestamped

		uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent


		void publish_mission_item() const;
		void publish_offboard_control_mode() const;
		void publish_position_setpoint_triplet() const;
		void publish_vehicle_command(uint32_t command, float param1 = 0.0,
						float param2 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	/*msg.timestamp = timestamp_.load();
	msg.ignore_thrust = true;
	msg.ignore_attitude = true;
	msg.ignore_bodyrate_x = true;
	msg.ignore_bodyrate_y = true;
	msg.ignore_bodyrate_z = true;
	msg.ignore_position = false;
	msg.ignore_velocity = true;
	msg.ignore_acceleration_force = true;
	msg.ignore_alt_hold = true;
	*/
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish position setpoint triplets.
 *        For this example, it sends position setpoint triplets to make the
 *        vehicle hover at 5 meters.
 */
void OffboardControl::publish_position_setpoint_triplet() const {
	PositionSetpointTriplet msg{};
	/*
	msg.timestamp = timestamp_.load();
	msg.current.timestamp = timestamp_.load();
	msg.current.type = PositionSetpoint::SETPOINT_TYPE_POSITION;
	msg.current.x = 0.0;
	msg.current.y = 0.0;
	msg.current.z = -5.0;
	msg.current.yaw = 1.5707963268;
	msg.current.cruising_speed = -1.0;
	msg.current.position_valid = true;
	msg.current.yaw_valid = true;
	msg.current.alt_valid = true;
	msg.current.valid = true;
	*/
	position_setpoint_triplet_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint32_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = 0.0;
	msg.param4 = NAN;
	msg.param5 = 47.397344;
	msg.param6 = 8.546152;
	msg.param7 = 50.0;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = false;
	

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::publish_mission_item() const
{
	NavigatorMissionItem msg{};
	msg.timestamp = timestamp_.load();
	msg.instance_count =0;
	msg.sequence_current =0;
	msg.nav_cmd=16; //takeoff
	msg.latitude = 47.3973;
	msg.longitude = 8.5461;
	msg.time_inside=0.0;
	msg.acceptance_radius=5.0;
	msg.loiter_radius=0.0;
	msg.yaw = 2.5;
	msg.altitude =55.0;
	msg.frame =3; 
	msg.origin=0;
	msg.loiter_exit_xtrack=false;
	msg.force_heading=false;
	msg.altitude_is_relative = true;
	msg.autocontinue =true;
	msg.vtol_back_transition=false;
	
	mission_item_publisher_->publish(msg);

}


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
