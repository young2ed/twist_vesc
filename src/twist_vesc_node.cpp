#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

class TWIST_VESC
{
public:
    TWIST_VESC(ros::NodeHandle &n)
    {
        ros::NodeHandle private_nh("~");

        if (!getRequiredParam(n, "max_nav_forward_speed", this->max_forward_speed))
            return;
        if (!getRequiredParam(n, "max_nav_reverse_speed", this->max_reverse_speed))
            return;
        if (!getRequiredParam(n, "vesc/wheelbase", this->wheelbase))
            return;

        // enable
        this->enable_sub = n.subscribe<std_msgs::Bool>("/teleop_enable", 0, &TWIST_VESC::subscribeEnabled, this);

        // cmd_sub
        this->cmd_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 0, &TWIST_VESC::subscribeTwist, this);

        // Ackermann
        private_nh.param<std::string>("pub_ackermann", this->pubAckermann, "/ackermann_cmd_mux/input/ackermann_twist");
        this->vesc_chat = n.advertise<ackermann_msgs::AckermannDriveStamped>(this->pubAckermann, 10);

        ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        ackermann_msg.header.frame_id = "base_link";
    }

    ~TWIST_VESC()
    {
        //
    }

    void subscribeEnabled(const std_msgs::Bool::ConstPtr &enabled)
    {
        this->is_enabled = !enabled->data;      //opposite enable of joystick
    }

    void subscribeTwist(const geometry_msgs::Twist::ConstPtr &twist)
    {
        // power
        this->power = sqrt(pow(twist->linear.x, 2.0) + pow(twist->linear.y, 2.0));
        if (twist->linear.x < 0) {
            if (this->power > this->max_reverse_speed)   this->power = this->max_reverse_speed;
            this->power *= -1;
        }
        else if (this->power > this->max_forward_speed){
            this->power = this->max_forward_speed;
        }

        // servo
		if(twist->linear.x == 0 || twist->angular.z == 0) {
            this->servo = 0;
        }
		else {
            this->servo = atan( this->wheelbase / (twist->linear.x / twist->angular.z));
        }
        this->servo = (float) this->servo / 5.2;    // map +-pi/2 to +-0.3
        if (this->power < 0) this->servo *= -1;     // negate angle if driving backwards
    }

    void run()
    {
        this->ackermann_msg.header.stamp = ros::Time::now();
        if (!this->is_enabled)
        {
            //this->publishZero();
        }
        else
        {
            this->publishControl();
        }
    }

    void publishZero()
    {
        this->ackermann_msg.drive.steering_angle = 0;
        this->ackermann_msg.drive.speed = 0;
        vesc_chat.publish(ackermann_msg);
    }

    void publishControl()
    {
		this->ackermann_msg.drive.steering_angle = (float) this->servo;
		this->ackermann_msg.drive.speed = (float) this->power;
        this->ackermann_msg.header.stamp = ros::Time::now();
        this->vesc_chat.publish(ackermann_msg);
    }

    template<typename T>
    inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value) {
        if (nh.getParam(name, value))
            return true;

        ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
        return false;
    }

private:
    //Topics
    ros::Publisher vesc_chat;
    ros::Subscriber enable_sub, cmd_sub;

    std::string pubAckermann;
    ackermann_msgs::AckermannDriveStamped ackermann_msg;

    bool is_enabled = false;
    double max_forward_speed, max_reverse_speed, wheelbase;
    double servo = 0;
    double power = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_vesc_node");
    ros::NodeHandle n;

    TWIST_VESC *twist_vesc = new TWIST_VESC(n);
    ros::Rate loop_rate(20);    //20 Hz

    while (ros::ok())
    {
        twist_vesc->run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete twist_vesc;
    return 0;
}
