
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>



class VehicleStatePubSub : public rclcpp::Node
{
    public:
        explicit VehicleStatePubSub() : Node("SensorCombined_listener")
        {
            vehicle_status_=this->create_subscription<px4_msgs::msg::SensorCombined>("SensorCombined_PubSubTopic",10,
            [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
                std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
                std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
                std::cout << "============================="   << std::endl;
                std::cout << "ts: "          << msg->timestamp    << std::endl;
                std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
                std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
                std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
                std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
                std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
                std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
                std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
                std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
                std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;

                });

        }            

        private:
            rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr vehicle_status_;
              
};



int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_status listener node..." << std::endl;
	rclcpp::init(argc, argv);
	auto statesub=std::make_shared<VehicleStatePubSub>();
    rclcpp::spin(statesub);

	rclcpp::shutdown();
	return 0;
}