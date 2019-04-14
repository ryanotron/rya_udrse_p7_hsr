#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef visualization_msgs::Marker VisMarker;

bool object_picked;
bool object_dropped;
double pickup_x;
double pickup_y;
double dropoff_x;
double dropoff_y;
double th;
VisMarker marker;

ros::Publisher marker_pub;

void odom_cb(const geometry_msgs::PoseWithCovarianceStamped odom) {
    // short circuit when all tasks are done
    if (object_dropped) {
        return;
    }

    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;

    double dx, dy;

    if (object_picked) {
        dx = x - dropoff_x;
        dy = y - dropoff_y;
    }
    else {
        dx = x - pickup_x;
        dy = y - pickup_y;
    }

    double dee = (dx*dx + dy*dy);
    ROS_INFO("distance: (%4.2f, %4.2f), (%4.2f, %4.2f): %4.2f", x, dx, y, dy, dee);
    if (dee < th) {
        if (!object_picked) {
            marker.color.a = 0.0f;
            marker_pub.publish(marker);
            object_picked = true;
            ROS_INFO("picking up object");
        }
        else if (!object_dropped) {
            marker.color.a = 1.0f;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker_pub.publish(marker);
            object_dropped = true;
            ROS_INFO("dropping off object");
        }
    }

}

int main(int argc, char** argv) {
    // node setup
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;

    // pubsub setup
    marker_pub = nh.advertise<visualization_msgs::Marker>("vis_marker", 1);
    ros::Subscriber odom_sub = nh.subscribe("amcl_pose", 1, odom_cb);

    // globals initialisation
    object_picked = false;
    object_dropped = false;

    // TODO: parametrise these
    pickup_x = 0.5;
    pickup_y = 0.5;
    dropoff_x = -2.0;
    dropoff_y = 2.0;
    th = 0.1;

    // main process setup
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "pick_object";
    marker.id = 0;

    marker.type = VisMarker::CUBE;
    marker.action = VisMarker::ADD;

    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
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

    // keep node alive
    while (ros::ok()) {
        ros::spinOnce();
    }

}
