#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

typedef visualization_msgs::Marker VisMarker;

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vis_marker", 1);

    VisMarker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "pick_object";
    marker.id = 0;

    marker.type = VisMarker::CUBE;
    marker.action = VisMarker::ADD;

    marker.pose.position.x = 1.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Got subsriber for the marker. Publishing...");

    // publish at pickup
    ROS_INFO("at pickup");
    marker_pub.publish(marker);

    ros::Duration(5.0).sleep();

    // hide marker
    ROS_INFO("away");
    marker.color.a = 0.0f;
    marker_pub.publish(marker);

    ros::Duration(5.0).sleep();

    // publish at dropoff
    ROS_INFO("at dropoff");
    marker.color.a = 1.0f;
    marker.pose.position.x = -1;
    marker_pub.publish(marker);

    // keep node alive
    while (ros::ok()) {
        ros::spinOnce();
    }

}
