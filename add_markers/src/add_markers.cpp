#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


double pickup_x=-2.0;
double pickup_y=2.0;

double drop_x=3.0;
double drop_y=3.0;


enum state {PICKING_UP,DROPING,DROPED};

state actual_state = PICKING_UP;
double distance(double x2,double y2,double x1,double y1){
    sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    //ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
  	double drop_dist=distance(pickup_x,msg->pose.pose.position.x,pickup_y,msg->pose.pose.position.y);
    double dist= distance(drop_x,msg->pose.pose.position.x,drop_y,msg->pose.pose.position.y);
    switch (actual_state)
    {    
        case PICKING_UP:
        	
            if(drop_dist<0.05){
                ROS_INFO("Move to droping %f",drop_dist);
                actual_state=DROPING;
            }
            break;
        case DROPING:
     
            if(dist<0.05){
                ROS_INFO("Move to droped %f",dist);
                actual_state=DROPED;
            }
            break;
    }
    //ROS_INFO(actual_state);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    ros::Subscriber sub = n.subscribe("/odom", 1000, odometry_callback);
    ROS_INFO("1");
    if(actual_state!=DROPED){
        ros::spin();
        ROS_INFO("2");
    }
    ROS_INFO("3");
    return 0;
}