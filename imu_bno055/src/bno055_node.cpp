#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "imu_bno055/bno055.hpp"
#include <yaml-cpp/yaml.h>

class BNO055Node : public rclcpp::Node {
public:
    BNO055Node() : Node("bno055_node") {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        calib_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("imu/calibration", 10);
        ypr_pub_ = create_publisher<std_msgs::msg::String>("imu/ypr", 10);

        sensor_ = std::make_shared<BNO055>(1);
        sensor_->begin();
        // Load offsets from YAML
	std::vector<int16_t> offsets;
	std::string yaml_path = 
    	this->declare_parameter<std::string>("offset_file", 
        "/home/pi/ros2_ws/src/imu_bno055/config/offsets.yaml");

	if (loadOffsetsFromYaml(yaml_path, offsets)) {
    		RCLCPP_INFO(get_logger(), "Loaded offsets from YAML, writing to IMU...");
    	if (sensor_->writeOffsets(offsets)) {
        	RCLCPP_INFO(get_logger(), "Offsets successfully written to BNO055");
    	} else {
        	RCLCPP_WARN(get_logger(), "Failed to write offsets to BNO055");
    	}
	} else {
    		RCLCPP_WARN(get_logger(), "Could not load offsets YAML, starting uncalibrated");
	}
        sensor_->setMode(0x0C);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&BNO055Node::update, this));
    }

private:
    void update() {
        float q[4], a[3], g[3], m[3];
        uint8_t sys, gyr, acc, mag;
        float yaw = 0.0, pitch = 0.0, roll = 0.0;

        sensor_->readQuaternion(q);
        sensor_->readAccel(a);
        sensor_->readGyro(g);
        sensor_->readMag(m);
        sensor_->readCalibration(sys, gyr, acc, mag);

	quaternionToEuler(q, yaw, pitch, roll);
	std_msgs::msg::String ypr_msg;
	ypr_msg.data = std::to_string(yaw) + " " +
		       std::to_string(pitch) + " " +
	               std::to_string(roll);
	ypr_pub_->publish(ypr_msg);
	
        auto imu = sensor_msgs::msg::Imu();
        imu.orientation.w = q[0];
        imu.orientation.x = q[1];
        imu.orientation.y = q[2];
        imu.orientation.z = q[3];

        imu.angular_velocity.x = g[0];
        imu.angular_velocity.y = g[1];
        imu.angular_velocity.z = g[2];

        imu.linear_acceleration.x = a[0];
        imu.linear_acceleration.y = a[1];
        imu.linear_acceleration.z = a[2];

        imu_pub_->publish(imu);

        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.magnetic_field.x = m[0];
        mag_msg.magnetic_field.y = m[1];
        mag_msg.magnetic_field.z = m[2];
        mag_pub_->publish(mag_msg);

        auto calib = std_msgs::msg::UInt8MultiArray();
        calib.data = {sys, gyr, acc, mag};
        calib_pub_->publish(calib);
    }

    bool loadOffsetsFromYaml(const std::string &path, std::vector<int16_t> &offsets){
    try {
        YAML::Node config = YAML::LoadFile(path);

        offsets.clear();
        offsets.reserve(11);

        auto accel = config["offsets"]["accel"];
        auto mag   = config["offsets"]["mag"];
        auto gyro  = config["offsets"]["gyro"];

        offsets.push_back(accel[0].as<int16_t>());
        offsets.push_back(accel[1].as<int16_t>());
        offsets.push_back(accel[2].as<int16_t>());

        offsets.push_back(mag[0].as<int16_t>());
        offsets.push_back(mag[1].as<int16_t>());
        offsets.push_back(mag[2].as<int16_t>());

        offsets.push_back(gyro[0].as<int16_t>());
        offsets.push_back(gyro[1].as<int16_t>());
        offsets.push_back(gyro[2].as<int16_t>());

        offsets.push_back(config["offsets"]["radius_accel"].as<int16_t>());
        offsets.push_back(config["offsets"]["radius_mag"].as<int16_t>());

        return true;
    }
    catch (const std::exception &e) {
        std::cerr << "Failed to load offsets YAML: " << e.what() << "\n";
        return false;
    }
}

	void quaternionToEuler(float q[4], float &yaw, float &pitch, float &roll){
	
	//yaw (Z axis)
	yaw = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
	
	//pitch (Y-axis)
	pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
	
	//roll (X-Axis)
	roll = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
	
	//Convert to degrees
	yaw = yaw * 180.0f / M_PI;
	pitch = pitch * 180.0f / M_PI;
	roll = roll * 180.0f / M_PI;
	}
	
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr calib_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ypr_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<BNO055> sensor_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNO055Node>());
    rclcpp::shutdown();
    return 0;
}
