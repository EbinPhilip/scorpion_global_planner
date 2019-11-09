 #ifndef SCORPION_GLOBAL_PLANNER_H
 #define SCORPION_GLOBAL_PLANNER_H

#include <string>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>

#include <a_star.h>

namespace scorpionPlanner
{
	class scorpionGlobalPlanner : public nav_core::BaseGlobalPlanner
	{
        public:

		scorpionGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, nav_msgs::Path *blockedPoints);

		bool makePlan(const geometry_msgs::PoseStamped& start, 
				const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
		
		bool makePlan(const geometry_msgs::PoseStamped& start, 
													const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
													double& cost);
													
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        private:
        std::string name;
        costmap_2d::Costmap2D *costmap;
		aStar searcher;
		nav_msgs::Path *blocked_points;
	};
};
#endif