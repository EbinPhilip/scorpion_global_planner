#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <string>
#include <scorpion_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

class globalPlannerExample
{
    public:
    globalPlannerExample(ros::NodeHandle *n):
    tfListener(tfBuffer),
    costmap("costmap", tfBuffer),
    planner("global", &costmap, &blockedPoints),
    nh(n)
    {
    }

    void callBack(const geometry_msgs::PoseStamped &pose)
    {
        path.poses.clear();
        blockedPoints.poses.clear();
        geometry_msgs::PoseStamped start;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "/map";
        blockedPoints.header.stamp = ros::Time::now();
        blockedPoints.header.frame_id = "/map";
        if (!planner.makePlan(start, pose, path.poses)) {
            ROS_ERROR("path planning failed!");
        }
        path_pub.publish(path);
        blocked_pub.publish(blockedPoints);
    }

    void run()
    {
        path_pub = nh->advertise<nav_msgs::Path>("/path", 1);
        blocked_pub = nh->advertise<nav_msgs::Path>("/blocked", 1);
        goal_sub = nh->subscribe("/move_base_simple/goal", 1, &globalPlannerExample::callBack, this);
        ROS_INFO("subscriber and publisher ready \n");
        ros::spin();
    }

    private:
    ros::NodeHandle *nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    costmap_2d::Costmap2DROS costmap;
    scorpionPlanner::scorpionGlobalPlanner planner;
    ros::Publisher path_pub;
    ros::Subscriber goal_sub;
    nav_msgs::Path path;
    nav_msgs::Path blockedPoints;
    ros::Publisher blocked_pub;
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "global_planner_demo");
    ros::NodeHandle nh;
    globalPlannerExample example(&nh);
    example.run();
}