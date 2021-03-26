#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


double pickup_x=-2.0;
double pickup_y=-2.0;

double drop_x=2.0;
double drop_y=2.0;


enum state {PICKING_UP,PUBLISH_TARGET,DROPING,DROPED};

state actual_state = PICKING_UP;
double distance(double x2,double x1,double y2,double y1){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
  	double pickup_dist=distance(pickup_x,msg->pose.pose.position.x,pickup_y,msg->pose.pose.position.y);
    double drop_dist= distance(drop_x,msg->pose.pose.position.x,drop_y,msg->pose.pose.position.y);
    //ROS_INFO("Pickup: %f  drop: %f position:x%f y%f",pickup_dist,drop_dist,msg->pose.pose.position.x,msg->pose.pose.position.y);
    //ROS_INFO_STREAM("state:"<<actual_state);
    switch (actual_state)
    {    
        case PICKING_UP:
            if(pickup_dist<0.5){
                ROS_INFO("Move to droping %f",pickup_dist);
                actual_state=PUBLISH_TARGET;
            }
            break;
        case DROPING:
            if(drop_dist<0.5){
                ROS_INFO("Move to droped %f",drop_dist);
                actual_state=DROPED;
            }
            break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(5);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    ros::Subscriber sub = n.subscribe("odom", 100, odometry_callback);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = "add_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
    marker.pose.position.z = 0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_INFO("Publish at pickup");
    marker_pub.publish(marker);

    while (ros::ok()){
        if(actual_state==PUBLISH_TARGET){
            ROS_INFO("Publish at drop zone");
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            r.sleep();
            marker.pose.position.x = pickup_x;
            marker.pose.position.y = pickup_y;
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            actual_state=DROPING;
        }else if(actual_state==DROPED){
            ROS_INFO("Droped");
            break;
        }
        ros::spinOnce();
    }
    ROS_INFO("Done");
    return 0;
}