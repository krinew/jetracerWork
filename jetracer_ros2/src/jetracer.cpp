#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity 0x11
#define sendType_params 0x12
#define sendType_coefficient 0x13

using namespace std;
using namespace boost::asio;
using std::placeholders::_1;
using namespace std::chrono_literals;

// SERIAL PROTOCOL HELPERS
uint8_t checksum(uint8_t *buf, size_t len)
{
    uint8_t sum = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        sum += *(buf + i);
    }
    return sum;
}

class JetRacerNode : public rclcpp::Node
{
public:
    JetRacerNode() : Node("jetracer"), iosev(), sp(iosev)
    {
        // 1. Declare and Get Parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<bool>("publish_odom_transform", true);
        this->declare_parameter<float>("linear_correction", 1.0);
        this->declare_parameter<float>("coefficient_a", -0.016073);
        this->declare_parameter<float>("coefficient_b", 0.176183);
        this->declare_parameter<float>("coefficient_c", -23.428084);
        this->declare_parameter<float>("coefficient_d", 1500.0);

        // PID params (formerly dynamic reconfigure)
        this->declare_parameter<int>("kp", 350);
        this->declare_parameter<int>("ki", 120);
        this->declare_parameter<int>("kd", 0);
        this->declare_parameter<int>("servo_bias", 0);

        this->get_parameter("port_name", port_name_);
        this->get_parameter("publish_odom_transform", publish_odom_transform_);

        // 2. Initialize Serial Port
        try
        {
            sp.open(port_name_);
            sp.set_option(serial_port::baud_rate(115200));
            sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
            sp.set_option(serial_port::parity(serial_port::parity::none));
            sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            sp.set_option(serial_port::character_size(8));
            RCLCPP_INFO(this->get_logger(), "Serial port opened at %s", port_name_.c_str());
        }
        catch (boost::system::system_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }

        // 3. Setup Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
        rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
        lset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
        rset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

        odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. Setup Subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&JetRacerNode::cmd_callback, this, _1));

        // 5. Send Initial Config
        sendInitialConfig();

        // 6. Start Serial Read Thread
        serial_thread_ = std::thread(std::bind(&JetRacerNode::serial_task, this));

        // 7. Start Control Loop Timer (50Hz -> 20ms)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&JetRacerNode::control_loop, this));

        cmd_time_ = this->now();
    }

    ~JetRacerNode()
    {
        if (serial_thread_.joinable())
            serial_thread_.detach(); // Or join depending on shutdown logic
        iosev.stop();
        sp.close();
    }

private:
    // ROS Members
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rset_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial Members
    io_service iosev;
    serial_port sp;
    std::thread serial_thread_;
    std::string port_name_;

    // Robot State
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    rclcpp::Time cmd_time_;
    bool publish_odom_transform_;

    void sendInitialConfig()
    {
        std::this_thread::sleep_for(20ms);

        // Read params
        float a = this->get_parameter("coefficient_a").as_double();
        float b = this->get_parameter("coefficient_b").as_double();
        float c = this->get_parameter("coefficient_c").as_double();
        float d = this->get_parameter("coefficient_d").as_double();

        SetCoefficient(a, b, c, d);

        int p = this->get_parameter("kp").as_int();
        int i = this->get_parameter("ki").as_int();
        int kd = this->get_parameter("kd").as_int();
        int servo_bias = this->get_parameter("servo_bias").as_int();
        float linear_correction = this->get_parameter("linear_correction").as_double();

        SetParams(p, i, kd, linear_correction, servo_bias);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        x_ = msg->linear.x;
        y_ = msg->linear.x; // NOTE: Original code mapped linear.x to both x and y. Preserving behavior.
        yaw_ = msg->angular.z;
        cmd_time_ = this->now();
    }

    void control_loop()
    {
        // Watchdog: If no command for 1 second, stop.
        if ((this->now() - cmd_time_).seconds() > 1.0)
        {
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
        }
        SetVelocity(x_, y_, yaw_);
    }

    // --- Serial Protocol Sending Functions ---
    void SetParams(int p, int i, int d, double linear_correction, int servo_bias)
    {
        uint8_t tmp[15];
        tmp[0] = head1;
        tmp[1] = head2;
        tmp[2] = 0x0F;
        tmp[3] = sendType_params;
        tmp[4] = (p >> 8) & 0xff;
        tmp[5] = p & 0xff;
        tmp[6] = (i >> 8) & 0xff;
        tmp[7] = i & 0xff;
        tmp[8] = (d >> 8) & 0xff;
        tmp[9] = d & 0xff;
        tmp[10] = (int16_t)((int16_t)(linear_correction * 1000) >> 8) & 0xff;
        tmp[11] = (int16_t)(linear_correction * 1000) & 0xff;
        tmp[12] = ((int16_t)((int16_t)servo_bias) >> 8) & 0xff;
        tmp[13] = ((int16_t)servo_bias) & 0xff;
        tmp[14] = checksum(tmp, 14);
        write(sp, buffer(tmp, 15));
        RCLCPP_INFO(this->get_logger(), "SetParams: p=%d i=%d d=%d lc=%f sb=%d", p, i, d, linear_correction, servo_bias);
    }

    void SetCoefficient(float a, float b, float c, float d)
    {
        uint8_t tmp[21];
        char *p;
        tmp[0] = head1;
        tmp[1] = head2;
        tmp[2] = 0x15;
        tmp[3] = sendType_coefficient;
        p = (char *)&a;
        tmp[4] = p[0];
        tmp[5] = p[1];
        tmp[6] = p[2];
        tmp[7] = p[3];
        p = (char *)&b;
        tmp[8] = p[0];
        tmp[9] = p[1];
        tmp[10] = p[2];
        tmp[11] = p[3];
        p = (char *)&c;
        tmp[12] = p[0];
        tmp[13] = p[1];
        tmp[14] = p[2];
        tmp[15] = p[3];
        p = (char *)&d;
        tmp[16] = p[0];
        tmp[17] = p[1];
        tmp[18] = p[2];
        tmp[19] = p[3];
        tmp[20] = checksum(tmp, 20);
        write(sp, buffer(tmp, 21));
        RCLCPP_INFO(this->get_logger(), "SetCoefficient: a=%f b=%f c=%f d=%f", a, b, c, d);
    }

    void SetVelocity(double x, double y, double yaw)
    {
        uint8_t tmp[11];
        tmp[0] = head1;
        tmp[1] = head2;
        tmp[2] = 0x0b;
        tmp[3] = sendType_velocity;
        tmp[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
        tmp[5] = ((int16_t)(x * 1000)) & 0xff;
        tmp[6] = ((int16_t)(y * 1000) >> 8) & 0xff;
        tmp[7] = ((int16_t)(y * 1000)) & 0xff;
        tmp[8] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
        tmp[9] = ((int16_t)(yaw * 1000)) & 0xff;
        tmp[10] = checksum(tmp, 10);
        write(sp, buffer(tmp, 11));
    }

    // --- Serial Protocol Receiving ---
    void serial_task()
    {
        enum frameState
        {
            State_Head1,
            State_Head2,
            State_Size,
            State_Data,
            State_CheckSum,
            State_Handle
        };
        frameState state = State_Head1;

        uint8_t frame_size, frame_sum;
        uint8_t data[50];

        double imu_list[9];
        double odom_list[6];
        rclcpp::Time now_time, last_time;
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Start receive message loop");

        while (rclcpp::ok()) // Using library ok() is thread safe enough typically, or check atomic bool
        {
            try
            {
                switch (state)
                {
                case State_Head1:
                    read(sp, buffer(&data[0], 1));
                    state = (data[0] == head1 ? State_Head2 : State_Head1);
                    break;
                case State_Head2:
                    read(sp, buffer(&data[1], 1));
                    state = (data[1] == head2 ? State_Size : State_Head1);
                    break;
                case State_Size:
                    read(sp, buffer(&data[2], 1));
                    frame_size = data[2];
                    state = State_Data;
                    break;
                case State_Data:
                    read(sp, buffer(&data[3], frame_size - 4)); // Read remaining data
                    state = State_CheckSum;
                    break;
                case State_CheckSum:
                    read(sp, buffer(&data[frame_size - 1], 1));
                    frame_sum = checksum(data, frame_size - 1);
                    state = (data[frame_size - 1] == frame_sum) ? State_Handle : State_Head1;
                    break;
                case State_Handle:
                    now_time = this->now();
                    processData(data, now_time, last_time, imu_list, odom_list);
                    last_time = now_time;
                    state = State_Head1;
                    break;
                }
            }
            catch (boost::system::system_error &e)
            {
                // Serial port closed or error
                break;
            }
        }
    }

    void processData(uint8_t *data, rclcpp::Time now_time, rclcpp::Time last_time, double *imu_list, double *odom_list)
    {
        // --- IMU Processing ---
        imu_list[0] = ((double)((int16_t)(data[4] * 256 + data[5])) / 32768 * 2000 / 180 * 3.1415);
        imu_list[1] = ((double)((int16_t)(data[6] * 256 + data[7])) / 32768 * 2000 / 180 * 3.1415);
        imu_list[2] = ((double)((int16_t)(data[8] * 256 + data[9])) / 32768 * 2000 / 180 * 3.1415);
        // Acc
        imu_list[3] = ((double)((int16_t)(data[10] * 256 + data[11])) / 32768 * 2 * 9.8);
        imu_list[4] = ((double)((int16_t)(data[12] * 256 + data[13])) / 32768 * 2 * 9.8);
        imu_list[5] = ((double)((int16_t)(data[14] * 256 + data[15])) / 32768 * 2 * 9.8);
        // Angle
        imu_list[6] = ((double)((int16_t)(data[16] * 256 + data[17])) / 10.0);
        imu_list[7] = ((double)((int16_t)(data[18] * 256 + data[19])) / 10.0);
        imu_list[8] = ((double)((int16_t)(data[20] * 256 + data[21])) / 10.0);

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = now_time;
        imu_msg.header.frame_id = "base_imu_link";
        imu_msg.angular_velocity.x = imu_list[0];
        imu_msg.angular_velocity.y = imu_list[1];
        imu_msg.angular_velocity.z = imu_list[2];
        imu_msg.linear_acceleration.x = imu_list[3];
        imu_msg.linear_acceleration.y = imu_list[4];
        imu_msg.linear_acceleration.z = imu_list[5];

        // Convert RPY to Quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, (imu_list[8] / 180 * 3.1415926));
        imu_msg.orientation = tf2::toMsg(q);

        imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
        imu_msg.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
        imu_msg.linear_acceleration_covariance = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
        imu_pub_->publish(imu_msg);

        // --- Odometry Processing ---
        odom_list[0] = ((double)((int16_t)(data[22] * 256 + data[23])) / 1000);
        odom_list[1] = ((double)((int16_t)(data[24] * 256 + data[25])) / 1000);
        odom_list[2] = ((double)((int16_t)(data[26] * 256 + data[27])) / 1000);

        // Velocity data (dx, dy, dyaw)
        odom_list[3] = ((double)((int16_t)(data[28] * 256 + data[29])) / 1000);
        odom_list[4] = ((double)((int16_t)(data[30] * 256 + data[31])) / 1000);
        odom_list[5] = ((double)((int16_t)(data[32] * 256 + data[33])) / 1000);

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = now_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = odom_list[0];
        odom_trans.transform.translation.y = odom_list[1];
        odom_trans.transform.translation.z = 0.0;

        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, odom_list[2]);
        odom_trans.transform.rotation = tf2::toMsg(odom_quat);

        if (publish_odom_transform_)
        {
            odom_broadcaster_->sendTransform(odom_trans);
        }

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = odom_list[0];
        odom_msg.pose.pose.position.y = odom_list[1];
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf2::toMsg(odom_quat);

        odom_msg.child_frame_id = "base_footprint";
        double dt = (now_time - last_time).seconds();
        if (dt > 0)
        {
            odom_msg.twist.twist.linear.x = odom_list[3] / dt;
            odom_msg.twist.twist.linear.y = odom_list[4] / dt;
            odom_msg.twist.twist.angular.z = odom_list[5] / dt;
        }

        odom_msg.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                     0, 1e-3, 1e-9, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 0.1};
        odom_msg.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};
        odom_pub_->publish(odom_msg);

        // --- Motor Debug Pubs ---
        std_msgs::msg::Int32 msg_int;
        msg_int.data = ((int16_t)(data[34] * 256 + data[35]));
        lvel_pub_->publish(msg_int);
        msg_int.data = ((int16_t)(data[36] * 256 + data[37]));
        rvel_pub_->publish(msg_int);
        msg_int.data = ((int16_t)(data[38] * 256 + data[39]));
        lset_pub_->publish(msg_int);
        msg_int.data = ((int16_t)(data[40] * 256 + data[41]));
        rset_pub_->publish(msg_int);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetRacerNode>());
    rclcpp::shutdown();
    return 0;
}
