#include "a_star.h"
#include <algorithm>
#include <ros/ros.h>

Node::Node(std::pair<int, int> n_index, std::pair<int, int> n_parent, float ng_cost, float nf_cost):
index(n_index),
parent(n_parent),
g_cost(ng_cost),
f_cost(nf_cost)
{}

Node::Node(std::pair<int, int> n_index):
index(n_index),
parent(std::make_pair(-1,-1)),
g_cost(0),
f_cost(0)
{}

bool Node::operator == (const Node &obj) const
{
    if (this->index == obj.index) {
        return true;
    } else {
        return false;
    }
}

bool Node::operator == (const std::pair<int, int> &obj) const
{
    if (this->index == obj) {
        return true;
    } else {
        return false;
    }
}

bool Node::operator < (const Node &obj) const
{
    if (this->f_cost == obj.f_cost) {
        return this->index < obj.index;
    } else {
        return this->f_cost < obj.f_cost;
    }
}

std::pair<int, int> Node::operator + (const std::pair<int, int> &obj) const
{
    return std::make_pair(index.first + obj.first, index.second + obj.second);
}

std::pair<int, int> Node::operator - (const std::pair<int, int> &obj) const
{
    return std::make_pair(index.first - obj.first, index.second - obj.second);
}

Node::operator std::pair<int, int> () 
{
    return this->index;
}

aStar::aStar(nav_msgs::Path *blockedPoints, int blocked, float cost_move):
goal(std::make_pair(0,0)),
start(std::make_pair(0,0)),
cost(-1),
goal_reached(false),
open_list(std::set<Node>()),
closed_list(std::unordered_set<Node, NodeHasher>()),
path(std::vector< std::pair<int, int> >()),
blocked_cost(blocked),
move_cost(cost_move),
blocked_points(blockedPoints)
{}

bool aStar::findPath(int goal_x, int goal_y, int start_x, int start_y, int rows, int columns, costmap_2d::Costmap2D *input_graph)
{
    goal.first = goal_x;
    goal.second = goal_y;
    start.first = start_x;
    start.second = start_y;  
    graph = input_graph;
    row = rows;
    column = columns;

    ROS_INFO("goal cost: %d",graph->getCost(goal_x,goal_y));

    if (goal.first>=row || goal.first<0 || goal.second>=column || goal.second<0 || isBlocked(goal)) {
        ROS_ERROR("invalid goal");
        return false;
    } else if (start.first>=row || start.first<0 || start.second>=column || start.second<0 || isBlocked(start)) {
        ROS_ERROR("invalid start point");
        return false;
    } else if (start == goal) {
        goal_reached = true;
        cost = 0;
        path.push_back(start);
        return true;
    }

    path.clear();
    goal_reached = false;

    float h_cost = getHeuristic(start.first, start.second);
    Node newNode = Node(start, std::make_pair(-1, -1), 0, h_cost);
    open_list.insert(newNode);

    while (!open_list.empty())
    {
        Node current_node = *open_list.begin();
        addNode(current_node + std::make_pair(0,-1), &current_node);
        addNode(current_node + std::make_pair(0, 1), &current_node);
        addNode(current_node + std::make_pair(1, 0), &current_node);
        addNode(current_node + std::make_pair(-1, 0), &current_node);
        addNode(current_node + std::make_pair(1, 1), &current_node);
        addNode(current_node + std::make_pair(-1,-1), &current_node);
        addNode(current_node + std::make_pair(1,-1), &current_node);
        addNode(current_node + std::make_pair(-1,1), &current_node);
        closed_list.insert(current_node);
        open_list.erase(open_list.find(current_node));
        if (goal_reached) {
            break;
        }
    }

    if (goal_reached) {
        std::unordered_set<Node, NodeHasher>::iterator it = closed_list.find(goal);
        cost = it->g_cost;
        while (it != closed_list.end())  
        {
            path.push_back(it->index);
            it = closed_list.find(it->parent);
        }
        std::reverse(path.begin(), path.end());
        open_list.clear();
        closed_list.clear();
        return true;
    } else {
        ROS_ERROR("Search complete! Goal not found");
        return false;
    }
}

bool aStar::getPath(std::vector< std::pair<int, int> > &get_path) {
    if (goal_reached) {
        get_path = path;
        return true;
    } else {
        return false;
    }
}

float aStar::getPathCost() const 
{
    return cost;
}

bool aStar::addNode(std::pair<int, int> node_position, Node *parent)
{
    if (goal_reached) {
        return true;
    } else if (node_position == parent->parent) {
        return false;
    } else if (node_position.first>=row || node_position.first<0 || node_position.second>=column || node_position.second<0) {
        return false;
    } else if ( isBlocked(node_position) ) {
        return false;
    }

    float g_cost = parent->g_cost + move_cost + graph->getCost(node_position.first, node_position.second);
    float h_cost = getHeuristic(node_position.first, node_position.second);
    Node newNode = Node(node_position, *parent, g_cost, g_cost + h_cost);
    
    if (newNode == goal) {
        goal_reached = true;
        closed_list.insert(newNode);
        return true;
    } else if (open_list.find(newNode) != open_list.end()) {
        std::set<Node>::iterator it= open_list.find(newNode);
        if (newNode.f_cost < it->f_cost) {
            // open_list.erase(it);
            open_list.insert(newNode);
            return true;
        } else {
            return false;
        }
    } else if (closed_list.find(newNode) != closed_list.end()) {
        std::unordered_set<Node, NodeHasher>::iterator it = closed_list.find(newNode);
        if (newNode.f_cost < it->f_cost) {
            closed_list.erase(it);
            open_list.insert(newNode);
            return true;
        } else {
            return false;
        }
    } else {
        open_list.insert(newNode);
        return true;
    }
}

float aStar::getHeuristic(int x, int y) const
{
    // return std::max(x - goal.first, y - goal.second);
    return sqrt( pow((x - goal.first), 2) + pow((y - goal.second), 2) );
}

bool aStar::isBlocked(std::pair<int, int> node_position)
{
    bool ret = (graph->getCost(node_position.first, node_position.second) >= blocked_cost);
    if (ret) {
        ros::Time time = ros::Time::now();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/base_link";
        pose.header.stamp = time;

        graph->mapToWorld(node_position.first, node_position.second, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1;

        blocked_points->poses.push_back(pose);
    }
    return ret;
}

