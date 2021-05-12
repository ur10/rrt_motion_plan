#include <rrt_motion_plan/rrt.h>

RRT::RRT(float _incremental_distance, float _bias, geometry_msgs::Point _start, geometry_msgs::Point _goal)
 : incremental_distance{ _incremental_distance }, bias{ _bias }, start{ _start }, goal{ _goal }
 {
     marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

     Node root_node;
     
     root_node.coordinate = start;
     root_node.parent = NULL;
     node_list.push_back( &root_node );

     ROS_INFO("Initializing RRT\n");
 }


 geometry_msgs::Point RRT::getRandomPoint()
 {
    int x_min, x_max;
    geometry_msgs::Point random_point;
    
    x_max = 18;
    x_min = -18;

    random_point.x = ( random() % ( x_max - x_min +1) ) + x_min;
    random_point.y = ( random() % ( x_max - x_min +1) ) + x_min;

    ROS_INFO("The random points are %f %f \n", random_point.x, random_point.y );

    return random_point;

 }

Node* RRT::getNearestPoint(geometry_msgs::Point point )
 {
     float distance = MAXFLOAT;
     Node* node_nearest;

    for(int i = 0; i < node_list.size(); i++)
    {
        if( distance > getDistance( point, node_list[i]->coordinate) )
        {
            distance = getDistance( point, node_list[i]->coordinate);
            node_nearest = node_list[i];
        }
    }

  ROS_INFO("The nearest point is %f %f \n", node_nearest->coordinate.x ,node_nearest->coordinate.y);

    return node_nearest;
 }

 Node* RRT::getNewConfig(geometry_msgs::Point nearest_point, geometry_msgs::Point random_point, std::vector<visualization_msgs::Marker> obstacle_list )
 {
     Node* new_node;
     float slope;
     float delta_x, delta_y;
     geometry_msgs::Point new_point;
     
     delta_x =  random_point.x - nearest_point.x;
     delta_y =  random_point.y - nearest_point.y;
     slope = std::atan2( delta_y, delta_x );

     new_point.x = nearest_point.x + incremental_distance*std::cos( slope );
     new_point.y = nearest_point.y + incremental_distance*std::sin( slope );
    
     if( obstacleDetect( new_point, nearest_point, obstacle_list ) == true ) {
         new_node = NULL;
         ROS_INFO("Detected obstacle \n");

     } else {
         new_node = new Node;

        if( new_node != NULL ) {
             new_node->coordinate = new_point;
        }
     }

  
    return new_node;
 }

void RRT::addEdge( Node* node_new, Node* node_near )
{
    static visualization_msgs::Marker edge, vertex;

    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;
    edge.scale.x = 0.04;
    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;


    if( node_new != NULL && node_near != NULL ) {
   
    node_new->parent = node_near;
    node_new->vertex_list.push_back(node_near);
    node_near->vertex_list.push_back(node_new);

    node_list.push_back(node_new);

    edge.points.push_back( node_near->coordinate );
    edge.points.push_back( node_new->coordinate );

    ROS_INFO("Adding edge between - %f %f and %f %f \n", node_near->coordinate.x, node_near->coordinate.y, node_new->coordinate.x, node_new->coordinate.y);
    marker_pub.publish(edge);
 
    }
}

bool RRT::obstacleDetect( geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obstacle_list )
{
        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;

        for (int i = 0; i < obstacle_list.size(); i++) {
            visualization_msgs::Marker obs = obstacle_list[i];

            float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5;
            float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5;
            float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5;
            float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5;

            //check for the bottom intersection
            bool bottom = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);
            //left intersect
            bool left = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);
            //right intersect
            bool right = lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);
            //top intersect
            bool top = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

            if (bottom || left || right || top) {
                return true;

            }
        }
        return false;
}

std::vector <Node*> RRT::drawPath( void )
{
    Node *node = NULL;
    std::vector <Node*> goal_path;
    static visualization_msgs::Marker edge, vertex;

    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;
    edge.scale.x = 0.08;
    edge.color.g = edge.color.r = 2;
    edge.color.a = 1.0;
    
    for(int i = 0 ; i < node_list.size(); i++)
    {
        if( getDistance( goal, node_list[i]->coordinate ) < 1 )
        {
            node = node_list[i];
            ROS_INFO("Found the goal node !!!! \n");
        }
        
    }

    if( node == NULL )
    {
        ROS_INFO(" Increase the iteration count, search is incomplete \n ");
    } else {

        while(  node->parent != NULL ) {
            if(marker_pub.getNumSubscribers() > 0 ) {
                edge.points.push_back( node->coordinate );
                edge.points.push_back( node->parent->coordinate );
                goal_path.push_back( node );

                marker_pub.publish( edge );
   
             }
        node = node->parent;
        }
    }

    return goal_path;
}

void RRT::move_bot( std::vector<Node*> goal_path )
{   
    float slope;

    static visualization_msgs::Marker rob;
    rob.type = visualization_msgs::Marker::CUBE;
    

    rob.header.frame_id = "map";
    rob.header.stamp = ros::Time::now();
    rob.ns = "rob";
    rob.id = 0;
    rob.action = visualization_msgs::Marker::ADD;
    rob.lifetime = ros::Duration();

    rob.scale.x = 0.5;
    rob.scale.y = 1;
    rob.scale.z = 0.25;
    rob.pose.orientation.w = 1;
    rob.pose.orientation.x = rob.pose.orientation.y = rob.pose.orientation.z = 0;
    rob.pose.position = goal_path[ goal_path.size() - 1 ]->coordinate;
    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    geometry_msgs::Point current_point, next_point;

    ROS_INFO("Moving the robot \n");
   
    for(int i = 0; i< goal_path.size() -1; i++ )
    {
        
        next_point = goal_path[ goal_path.size() - i - 1 ]->coordinate;

        slope = std::atan2( ( next_point.y - current_point.y ), (next_point.x - current_point.x ) );

        rob.pose.position = next_point;
        rob.pose.orientation.z = slope;
        
        marker_pub.publish( rob );
    }

    ros::shutdown();
}

bool RRT::lineIntersect( float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4 )
{

        // calculate the distance to intersection point
        float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {

            float intersectionX = x1 + (uA * (x2 - x1));
            float intersectionY = y1 + (uA * (y2 - y1));

            return true;
        }
        return false;
}

float RRT::getDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
    float distance;

    distance = std::sqrt( std::pow( p1.x - p2.x, 2 ) + std::pow(p1.y - p2.y, 2 ) );

    return distance;
}