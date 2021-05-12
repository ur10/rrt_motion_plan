#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <rrt_motion_plan/rrt.h>

void addObstacle(ros::Publisher marker_pub);
void initRviz(ros::Publisher marker_pub);

double obstacle_num;
std::vector<std::vector<double>> obstacle_scale(3);
std::vector<std::vector<double>> obstacle_pose(3);
std::vector<int> start_pose(2);
std::vector<int> goal_pose(2);
float incremental_distance;
float bias;
float iter_count;

static std::vector<visualization_msgs::Marker> obsVec;

int main(int argc, char** argv)
{
    ros::init( argc, argv, "RRT" );

    ros::NodeHandle nh;

    ros::Rate loop_rate(20);

    
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

    ROS_INFO("publishing to visual marker\n");

    nh.getParam("/rrt_configuration/obstacle_config/obstacle_num", obstacle_num);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_position_x", obstacle_pose[0]);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_position_y", obstacle_pose[1]);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_position_z", obstacle_pose[2]);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_scale_x", obstacle_scale[0]);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_scale_y", obstacle_scale[1]);
    nh.getParam("/rrt_configuration/obstacle_config/obstacle_scale_z", obstacle_scale[2]);
    nh.getParam("/rrt_configuration/rrt_config/goal_pose", goal_pose);
    nh.getParam("/rrt_configuration/rrt_config/start_pose", start_pose);
    nh.getParam("/rrt_configuration/rrt_config/incremental_distance", incremental_distance);
    nh.getParam("/rrt_configuration/rrt_config/bias", bias);
    nh.getParam("/rrt_configuration/rrt_config/iteration_count", iter_count);
        
    Node node_start;

    geometry_msgs::Point start, goal;

    start.x = start_pose[0];
    start.y = start_pose[1];
    goal.x = goal_pose[0];
    goal.y = goal_pose[1];


   // initialize RRT.
   RRT rrt{ 1.5, 0.5, start, goal };

   Node *node;
   node = &node_start;
   std::vector<Node*> goal_path;

      for( int i = 0; i < iter_count; i++)
      {

          geometry_msgs::Point random_point;
          Node* node_near;
          Node* node_new;
         float random_num;

          initRviz(marker_pub);
          addObstacle(marker_pub);
        
          random_point = rrt.getRandomPoint();
          node_near = rrt.getNearestPoint( random_point );

          random_num = rand() / (double) RAND_MAX;

          if( random_num > rrt.bias){
              node_new = rrt.getNewConfig( node_near->coordinate, goal, obsVec );
          }
          else{
              node_new = rrt.getNewConfig( node_near->coordinate, random_point, obsVec );
          }

          rrt.addEdge( node_new, node_near );
 
      }

          goal_path =  rrt.drawPath( );
      
        ros::shutdown();

}

void initRviz(ros::Publisher marker_pub)
{
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    v_start.header.frame_id = v_end.header.frame_id = "map";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    v_start.scale.x = v_start.scale.y = 0.2;
    v_end.scale.x = v_end.scale.y = 0.2;

    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;

    geometry_msgs::Point ps, pe;
    ps.x = start_pose[0];
    ps.y = start_pose[1];
    pe.x = goal_pose[0];
    pe.y = goal_pose[1];
    v_start.points.push_back(ps);
    v_end.points.push_back(pe);

    //publish edge and vertices
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);


}

void addObstacle(ros::Publisher marker_pub)
{
      std::vector< visualization_msgs::Marker> obs(obstacle_num);

      for(int i = 0; i < obstacle_num; i++)
      {
          obs[i].type = visualization_msgs::Marker::CUBE;
          obs[i].header.frame_id = "map";
          obs[i].header.stamp = ros::Time::now();
          obs[i].ns = "obstacles";
          obs[i].lifetime = ros::Duration();
          obs[i].action = visualization_msgs::Marker::ADD;
          obs[i].id = i-1;

          obs[i].scale.x = obstacle_scale[0][i];
          obs[i].scale.y = obstacle_scale[1][i];
          obs[i].scale.z = obstacle_scale[2][i];

          obs[i].pose.position.x = obstacle_pose[0][i];
          obs[i].pose.position.y = obstacle_pose[1][i];
          obs[i].pose.position.z = obstacle_pose[2][i];

          obs[i].pose.orientation.x = 0.0;
          obs[i].pose.orientation.y = 0.0;
          obs[i].pose.orientation.z = 0.0;
          obs[i].pose.orientation.w = 1.0;

          obs[i].color.a = 1;
          obs[i].color.r = obs[i].color.g = obs[i].color.b = 6.6f;

          marker_pub.publish(obs[i]);
        
          obsVec.push_back(obs[i]);
       }


}