#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "global_planner_demo_tf_publisher");
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "map";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.rotation.w = 1;
    ros::Rate t(10);
    while (ros::ok())
    {
        trans.header.stamp = ros::Time::now();
        br.sendTransform(trans);
        t.sleep();
    }
}