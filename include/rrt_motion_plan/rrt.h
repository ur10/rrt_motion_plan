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

class Node  // Maybe use Node class as a friend class, to avoid public access of its members?
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

geometry_msgs::Point getRandomPoint ( );
Node* getNearestPoint ( geometry_msgs::Point point );
Node* getNewConfig ( geometry_msgs::Point nearest_point, geometry_msgs::Point random_point, std::vector<visualization_msgs::Marker> obstacle_list );
void addEdge ( Node* node_new, Node* node_near );
bool obstacleDetect ( geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obstacle_list );
std::vector<Node*> drawPath ( void );
void move_bot (std::vector<Node*> goal_path);
float getDistance ( geometry_msgs::Point p1, geometry_msgs::Point p2 );

geometry_msgs::Point start;
geometry_msgs::Point goal;
geometry_msgs::Point min_point;
geometry_msgs::Point max_point;
float incremental_distance;
float bias;

private:

ros::NodeHandle nh;
ros::Publisher marker_pub;
std::vector <Node*> node_list;
Node* node_init;

bool lineIntersect ( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4 );


};

#endif // RRT_H