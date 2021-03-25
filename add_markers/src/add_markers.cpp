#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


int pickup_x=0;
int pickup_y=0;

int drop_x=5;
int drop_y=5;

enum state {PICKING_UP,DROPING,DROPED};

state actual_state = PICKING_UP;
double distance(double x2,double y2,double x1,double y1){
    sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    auto pos = msg->pose.pose.position; 
    ROS_INFO("Position-> x: [%f], y: [%f]", pos.x,pos.y);
    auto x = pos.x;
    auto y = pos.y;
    switch (actual_state)
    {
        case PICKING_UP:
            if(distance(pickup_x,x,pickup_y,y)<0.5){
                ROS_INFO("Move to droping");
                actual_state=DROPING;
            }
            break;
        case DROPING:
            if(distance(drop_x,x,drop_y,y)<0.5){
                ROS_INFO("Move to droped");
                actual_state=DROPED;
            }
            break;
    }
    ROS_INFO("end");
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