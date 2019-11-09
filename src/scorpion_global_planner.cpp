#include <scorpion_global_planner.h>
#include <vector>
#include <utility>

namespace scorpionPlanner
{
    scorpionGlobalPlanner::scorpionGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, nav_msgs::Path *blockedPoints):
    searcher(aStar(blockedPoints, 128, 1.0f)),
    blocked_points(blockedPoints)
    {
        this->initialize(name, costmap_ros);
    }

    void scorpionGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        this->name = name;
        this->costmap = costmap_ros->getCostmap();
    }

    bool scorpionGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
				const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        int start_x;
        int start_y;
        int goal_x;
        int goal_y;
        costmap->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_x, start_y);
        costmap->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
        ROS_INFO("from: %d, %d to: %d, %d", start_x, start_y, goal_x, goal_y);
        if ( searcher.findPath(goal_x, goal_y, start_x, start_y,\
         costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap) ) {
            std::vector< std::pair<int, int> > path;
            searcher.getPath(path);
            ros::Time time = ros::Time::now();
            for (std::vector< std::pair<int, int> >::iterator i = path.begin(); i != path.end(); ++i)
            {
                double x,y;
                costmap->mapToWorld(i->first, i->second, x, y);
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "/base_link";
                pose.header.stamp = time;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
            }
            return true;
        } else {
            return false;
        }
    }

    bool scorpionGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
													const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
													double& cost)
    {
        bool flag = makePlan(start, goal, plan);
        cost = searcher.getPathCost();
        return flag;
    }
};