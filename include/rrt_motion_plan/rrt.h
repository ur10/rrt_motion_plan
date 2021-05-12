#ifndef RRT_H
#define RRT_H

#include "ros/ros.h"
#include "ros/subscriber.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <random>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

/**
 * \brief The instances of this class represent a unique point in the space.
*/
class Node
{
public:
 geometry_msgs::Point coordinate;
 std::vector <Node* > vertex_list;
 Node* parent;
};

class RRT
{
public:

RRT(float _incremental_distance, float _bias, geometry_msgs::Point _start, geometry_msgs::Point _goal);

/**
 * \brief Get a random point in the configuration space
*/
geometry_msgs::Point getRandomPoint ( );

/**
 * \brief Get the nearest point in the tree from a given random point
 * \param point Given random point
*/
Node* getNearestPoint ( geometry_msgs::Point point );

/**
 * \brief Get a new point in the configuration space del_q distance along the line between the nearest point and a random point
 * \param nearest_point The point nearest to a given random point.
 * \param random_point A random point in the configuration space.
 * \param obstacle_list This list contains the relevant obstacle info
*/
Node* getNewConfig ( geometry_msgs::Point nearest_point, geometry_msgs::Point random_point, std::vector<visualization_msgs::Marker> obstacle_list );

/**
 *  \brief Add edge between the new node and the nearest node
*/
void addEdge ( Node* node_new, Node* node_near );

/**
 * \brief Draw the final resultant path and get the path
 *  
*/
std::vector<Node*> drawPath ( void );

/**
 * \brief Move a symbolic bot on the obtained path
 * \param goal_path The final obtained path
*/
void move_bot (std::vector<Node*> goal_path);

/**
 * \brief The relevant points for handlig the rrt config
*/

geometry_msgs::Point start;
geometry_msgs::Point goal;
geometry_msgs::Point min_point;
geometry_msgs::Point max_point;

/**
 * \brief The delta_q distance to be moved along the line between the nearest point and a random point
*/
float incremental_distance;

/**
 *  \brief  The bias in moving towards the goal point.
*/
float bias;

private:

ros::NodeHandle nh;
ros::Publisher marker_pub;

/**
 * \brief This vector keeps track of all the nodes added to the tree.
*/
std::vector <Node*> node_list;

/**
 * The root node of the tree.
*/
Node* node_init;

/**
 * \brief Check for obstacles along the line between two points.
 * \param p1 The first point
 * \param p2 The second point
 * \param obstacle_list This list contains the relevant obstacle info
*/
bool obstacleDetect ( geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obstacle_list );

bool lineIntersect ( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4 );

/**
 * \brief Get the eucledian distance between two points.
*/
float getDistance ( geometry_msgs::Point p1, geometry_msgs::Point p2 );


};

#endif // RRT_H