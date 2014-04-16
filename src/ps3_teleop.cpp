#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>


#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

class TeleopPS3Car
{
public:
    TeleopPS3Car();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient disable, enable, trackOn, trackOff;
    bool run, car_like, track;
    double turning_radius, quot;
    double last_linear, last_angular;

};


TeleopPS3Car::TeleopPS3Car()
{
    // Default settings
    run = true;
    car_like = false;
    track = false;
    turning_radius = 1;
    last_angular = 0;
    last_linear = 0;

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("car_like", car_like, car_like);
    nh_.param("turning_radius", turning_radius, turning_radius);

    quot = 1./(2*turning_radius);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ps3/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopPS3Car::joyCallback, this);

    disable = nh_.serviceClient<std_srvs::Empty>("/ps3/disable_motors");
    enable = nh_.serviceClient<std_srvs::Empty>("/ps3/enable_motors");
    trackOn = nh_.serviceClient<std_srvs::Empty>("/tracking/start_tracking");
    trackOff = nh_.serviceClient<std_srvs::Empty>("/tracking/stop_tracking");

}

void TeleopPS3Car::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;


	vel.linear.x = l_scale_*joy->axes[linear_];
	vel.angular.z = a_scale_*joy->axes[angular_];
	if(vel.linear.x < 0){
		vel.angular.z *= -1;
	}
	if(last_angular != 0 && last_linear != 0)
		vel_pub_.publish(vel);
    last_angular =  vel.angular.z;
    last_linear = vel.linear.x;


    if(joy->buttons[PS3_BUTTON_ACTION_CROSS] && run){
        run = !run;
        std_srvs::Empty srv;
        disable.call(srv);
        ROS_WARN("Emergency Stop!");
        trackOff.call(srv);
        ROS_WARN("Tracking off!");
        track = false;
    } else if (joy->buttons[PS3_BUTTON_ACTION_CIRCLE] && !run) {
        run = !run;
        std_srvs::Empty srv;
        enable.call(srv);
        ROS_INFO("Motors on");
    } else if (joy->buttons[PS3_BUTTON_ACTION_TRIANGLE] && run && !track) {
    	track = !track;
        std_srvs::Empty srv;
        trackOn.call(srv);
        ROS_INFO("Tracking on");
    } else if (joy->buttons[PS3_BUTTON_ACTION_SQUARE] && run && track) {
    	track = !track;
        std_srvs::Empty srv;
        trackOff.call(srv);
        ROS_INFO("Tracking off");
    }


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ps3_teleop");
    TeleopPS3Car teleop_PS3;

    ros::spin();
}

