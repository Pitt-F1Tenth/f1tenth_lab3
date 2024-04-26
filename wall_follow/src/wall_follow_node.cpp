#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "diagnostics_msgs/msg/wall_following_diagnostics.hpp"

class WallFollow : public rclcpp::Node {
   public:
    WallFollow() : Node("wall_follow_node") {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1,
            std::bind(&WallFollow::odomCB, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            std::bind(&WallFollow::scanCB, this, std::placeholders::_1));
        autonomy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1,
            std::bind(&WallFollow::autonomyCB, this, std::placeholders::_1));

        // Publishers
        drive_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 10);
        diag_pub_ =
            this->create_publisher<diagnostics_msgs::msg::WallFollowingDiagnostics>(
                "/wall_follow_diag", 10);

        // Timer
        param_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&WallFollow::Timer, this));

        // Tuning Parameters
        this->declare_parameter("kp", 1.);
        kp_ = this->get_parameter("kp").as_double();
        this->declare_parameter("ki", 0.);
        ki_ = this->get_parameter("ki").as_double();
        this->declare_parameter("kd", 0.);
        kd_ = this->get_parameter("kd").as_double();
        this->declare_parameter("look_ahead", 1.0);
        look_ahead_ = this->get_parameter("look_ahead").as_double();
        this->declare_parameter("dist_from_wall", 1.0);
        dist_from_wall_ = this->get_parameter("dist_from_wall").as_double();
        this->declare_parameter("follow_right", false);
        follow_right_ = this->get_parameter("follow_right").as_bool();
    }

    void Timer() {
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double look_ahead = this->get_parameter("look_ahead").as_double();
        double dist_from_wall = this->get_parameter("dist_from_wall").as_double();
        bool follow_right = this->get_parameter("follow_right").as_bool();

        // Print updates
        if (dist_from_wall != dist_from_wall_) {
            RCLCPP_INFO(this->get_logger(), "Received change to dist_from_wall to %lf", dist_from_wall);
        }
        if (follow_right != follow_right_) {
            std::string follow_right_str = follow_right ? "right" : "left";
            RCLCPP_INFO(this->get_logger(), "Received change to follow %s wall", follow_right_str.c_str());
        }
        

        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        look_ahead_ = look_ahead;
        dist_from_wall_ = dist_from_wall;
        follow_right_ = follow_right;
    }

   private:
    // PID
    double kp_;
    double kd_;
    double ki_;

    double look_ahead_;      // [m]
    double dist_from_wall_;  // [m]
    double servo_offset = 0.0;
    double prev_error_ = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double derivative = 0.;
    bool autonomy_enabled_ = false;
    diagnostics_msgs::msg::WallFollowingDiagnostics diag_msg;

    // Odom
    nav_msgs::msg::Odometry odom;

    // Time
    rclcpp::Time prev_time;
    double dt_;

    // Angle
    bool follow_right_;
    bool scan_msg_rcvd = false;
    double angle_min;   // [rad]
    double angle_max;   // [rad]
    double angle_incr;  // [rad]
    double theta_ = deg2rad(40);        // [rad]

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr autonomy_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        drive_pub_;
    rclcpp::Publisher<diagnostics_msgs::msg::WallFollowingDiagnostics>::SharedPtr
        diag_pub_;
    rclcpp::TimerBase::SharedPtr param_timer_;

    void autonomyCB(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        autonomy_enabled_ = msg->buttons[3];
    }

    double get_range(std::vector<float> range_data, double& angle) {
        /*
        Simple helper to return the corresponding range measurement at a given
        angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle [rad]: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
        int range_idx = static_cast<int>((angle - angle_min) / angle_incr);
        double range = -1;

        // inf/nan check
        if (!std::isfinite(range_data[range_idx])) {
            int ccw_idx = 0;  // Traverse CCW
            while (!std::isfinite(range_data[range_idx + ccw_idx])) {
                ccw_idx++;
            }
            int cw_idx = 0;  // Traverse CW
            while (!std::isfinite(range_data[range_idx + cw_idx])) {
                cw_idx--;
            }
            // Find range
            int idx_shift = abs(ccw_idx) > abs(cw_idx) ? cw_idx : ccw_idx;
            range = range_data[range_idx + idx_shift];

            // Update angle too!
            angle = static_cast<int>((range_idx + idx_shift) * angle_incr +
                                     angle_min);
        } else {
            range = range_data[range_idx];
        }

        return range;
    }

    void pid_control(double error, double velocity) {
        /*
        Based on the calculated error, publish vehicle control
        K_p*e + K_i/s*e + K_d*s*e

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        if (ki_ > 0. && autonomy_enabled_) {
            integral += error * dt_;
        } else {
            integral = 0.0;
        }
        derivative = (error - prev_error_) / dt_;
        double angle = kp_ * error + ki_ * integral + kd_ * derivative;

        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;

        drive_pub_->publish(drive_msg);
        prev_error_ = error;

        // Diagnostics
        diag_msg.header.stamp = this->get_clock()->now();
        diag_msg.kp = kp_;
        diag_msg.ki = ki_;
        diag_msg.kd = kd_;
        diag_msg.p = kp_*error;
        diag_msg.i = ki_*integral;
        diag_msg.d = kd_*derivative;
        diag_msg.error = error;
        diag_msg.angle_command = angle;
        diag_pub_->publish(diag_msg);
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom = *msg;
    }

    void scanCB(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        /*
        Callback function for LaserScan messages. Calculate the error and
        publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        std::vector<float> scans = scan_msg->ranges;

        if (!scan_msg_rcvd) {
            // Read in sensor angle values for first time
            angle_min = scan_msg->angle_min;
            angle_max = scan_msg->angle_max;
            angle_incr = scan_msg->angle_increment;
            prev_time = this->get_clock()->now();
            scan_msg_rcvd = true;
        }

        dt_ = secs(rclcpp::Time(scan_msg->header.stamp) - prev_time);

        //! Calculate orthogonal angle from wall
        float dir = follow_right_ ? 1 : -1;
        float theta = dir * theta_;
        float dist_from_wall = dir * dist_from_wall_;
        double b_angle = deg2rad(-1 * dir * 90);         // [rad]
        double b = get_range(scans, b_angle);  // [rad]

        // Update a_angle and theta, in case of automatic angle shift
        double a_angle = b_angle + theta;
        double a = get_range(scans, a_angle);  // [rad]
        theta = a_angle - b_angle;
        
        // Orthogonal angle from wall
        double alpha =
            std::atan(a * std::cos(theta) - b) / (a * std::sin(theta));
        double d_t = dir*(b * std::cos(alpha));
        double d_t1 = d_t + look_ahead_ * std::sin(alpha);

        //! Compute Control Error
        double error = dist_from_wall - d_t1;

        //! Vehicle command
        double velocity = 0.0;

        diag_msg.a = a;
        diag_msg.b = b;
        diag_msg.a_angle = a_angle;
        diag_msg.b_angle = b_angle;
        diag_msg.theta = theta;
        diag_msg.alpha = alpha;
        diag_msg.target = dist_from_wall;
        diag_msg.dist = d_t1;
        diag_msg.follow_right = follow_right_;

        if (abs(error) < deg2rad(10)) {
            velocity = 1.5;
        } else if (abs(error) < deg2rad(20)) {
            velocity = 1.0;
        } else {
            velocity = 0.5;
        }
        pid_control(error, velocity);
    }

    float deg2rad(float angle) { return angle * M_PI / 180; }

    double secs(rclcpp::Time t) {
        return static_cast<double>(t.seconds()) +
               static_cast<double>(t.nanoseconds()) * 1e-9;
    }
    double secs(rclcpp::Duration t) {
        return static_cast<double>(t.seconds()) +
               static_cast<double>(t.nanoseconds()) * 1e-9;
    }
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
