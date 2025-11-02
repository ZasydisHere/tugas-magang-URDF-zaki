#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

// pANJANG Link (meter)
double a1 = 0.40, a2 = 0.30, a3 = 0.25;

ros::Publisher joint_pub;

void targetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    //YAW
    double theta1 = atan2(y, x);


    double r = sqrt(x*x + y*y);
    double d = sqrt(r*r + z*z);

    double maxReach = a1 + a2 + a3;
    if (d > maxReach) d = maxReach;

    double d_adj = d - a3;

    // IK 
    double cos2 = (d_adj*d_adj - a1*a1 - a2*a2) / (2*a1*a2);
    if (cos2 > 1) cos2 = 1;
    if (cos2 < -1) cos2 = -1;
    double sin2 = sqrt(1 - cos2*cos2);

    double theta3 = atan2(sin2, cos2);
    double theta2 = atan2(z, r) - atan2(a2*sin2, a1 + a2*cos2);

    // FK
    double px = (a1*cos(theta2) + a2*cos(theta2 + theta3) + a3*cos(theta2 + theta3)) * cos(theta1);
    double py = (a1*cos(theta2) + a2*cos(theta2 + theta3) + a3*cos(theta2 + theta3)) * sin(theta1);
    double pz = (a1*sin(theta2) + a2*sin(theta2 + theta3) + a3*sin(theta2 + theta3));

    ROS_INFO_STREAM(
        "\nTarget XYZ : (" << x << ", " << y << ", " << z << ")"
        << "\nFK XYZ     : (" << px << ", " << py << ", " << pz << ")"
        << "\nError XYZ  : (" << x-px << ", " << y-py << ", " << z-pz << ")"
        << "\nAngles(rad): yaw=" << theta1 << ", pitch1=" << theta2 << ", pitch2=" << theta3
    );

    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.name = {"shoulder", "elbow", "wrist"};
    joint_msg.position = {theta1, theta2, theta3};
    joint_pub.publish(joint_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Subscriber sub = nh.subscribe("/target_pose", 10, targetCallback);

    ROS_INFO("Yaw-Pitch-Pitch IK Controller running. Publish to /target_pose {x,y,z}");

    ros::Duration(0.5).sleep(); 

    sensor_msgs::JointState init_msg;
    init_msg.header.stamp = ros::Time::now();
    init_msg.name = {"shoulder", "elbow", "wrist"};
    init_msg.position = {0.0, 0.0, 0.0};
    joint_pub.publish(init_msg);

    ros::spin();
    return 0;
}
