#ifndef DEF_ASTAR
#define DEF_ASTAR

#include <vector>
#include <unordered_set>
#include <set>
#include <cmath>
#include <functional>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>


class NodeHasher;

class Node
{
    public:
    Node(std::pair<int, int> n_index, std::pair<int, int> n_parent, float ng_cost, float nf_cost);
    Node(std::pair<int, int> n_index);

    bool operator == (const Node &obj) const;
    bool operator == (const std::pair<int, int> &obj) const;
    bool operator < (const Node &obj) const;
    std::pair<int, int> operator + (const std::pair<int, int> &obj) const;
    std::pair<int, int> operator - (const std::pair<int, int> &obj) const;
    operator std::pair<int, int> ();

    std::pair<int, int> index;
    std::pair<int, int> parent;
    float g_cost;
    float f_cost;
};

class NodeHasher 
{
    public:
    std::size_t operator()(const Node &obj) const
    {
        return std::hash<int>()(obj.index.first+10000) ^ std::hash<int>()(obj.index.second-1000);
    }
};

class aStar
{
    public:
    aStar(nav_msgs::Path *blockedPoints, int blocked = 1, float cost_move = 1.0);
    bool findPath(int goal_x, int goal_y, int start_x, int start_y, int rows, int columns, costmap_2d::Costmap2D *input_graph);
    bool getPath(std::vector< std::pair<int, int> > &get_path);
    float getPathCost() const;

    private:
    std::pair<int, int> goal;
    std::pair<int, int> start;
    float cost;
    std::set<Node> open_list;
    std::unordered_set<Node, NodeHasher> closed_list;
    std::vector< std::pair<int, int> > path;
    // unsigned char *graph;
    costmap_2d::Costmap2D *graph;
    int row;
    int column;
    bool goal_reached;
    int blocked_cost;
    float move_cost;

    bool addNode(std::pair<int, int> node_position, Node *parent);
    float getHeuristic(int x, int y) const;
    bool isBlocked(std::pair<int, int> node_position);
	nav_msgs::Path *blocked_points;
};

#endif