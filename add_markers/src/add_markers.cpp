#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

using namespace std;

// shorthands
typedef visualization_msgs::Marker VisMarker;
typedef geometry_msgs::Point Pt;

ros::Publisher marker_pub;

vector<VisMarker> markers;
double theta_carry = 1.507;
double dtheta = 0.754;

bool object_picked;
bool object_dropped;
double pickup_x;
double pickup_y;
double dropoff_x;
double dropoff_y;

VisMarker marker;

// parameter containers
vector<double> dropoff;
vector< vector<double> > pickups;
double t_wait; // robot wait time when doing pickup
double th; // distance threshold

// flags
bool pickup_timer_trigger = false;
ros::Time pickup_time;

// bookkeeping
vector<bool> picked_up_status;

void add_pickup_cb(const Pt point) {
    VisMarker new_marker;
    new_marker.pose.position.x = point.x;
    new_marker.pose.position.y = point.y;
    new_marker.pose.position.z = point.z;

    new_marker.pose.orientation.w = 1.0;
    new_marker.header.frame_id = "map";

    markers.push_back(new_marker);
}

// draw markers at rate independent of pose message
void draw_markers() {
    for (int i = 0; i < markers.size(); i++) {
        ROS_INFO("marker %d frame %s", i, markers[i].header.frame_id.c_str());
        markers[i].header.stamp = ros::Time::now();
        marker_pub.publish(markers[i]);
    }
}

// calculate squared distance between two points
// no point (ha!) sqrooting the result
double calc_d(Pt pose, vector<double> ref) {
    double dx = pose.x - ref[0];
    double dy = pose.y - ref[1];
    return dx*dx + dy*dy;
}

void odom_cb(const geometry_msgs::PoseWithCovarianceStamped odom) {
    // short circuit if there is no marker left
    // that is, when we have returned to dropoff
    if (markers.size() < 1) {
        return;
    }

    // check if at dropoff
    double dee = calc_d(odom.pose.pose.position, dropoff);
    if (dee < th) {
        if (!pickup_timer_trigger) {
            pickup_timer_trigger = true;
            pickup_time = ros::Time::now();
        }
        else {
            double dt = (ros::Time::now() - pickup_time).toSec();
            if (dt > 0.5*t_wait) {
                // drop all picked up objects
                double drop_x, drop_y;
                drop_x = odom.pose.pose.position.x - 0.25; // add offset so the marker isn't occluded by the robot model in rviz
                drop_y = odom.pose.pose.position.y;

                int i = 0;
                while (i < markers.size()) {
                    if (markers[i].header.frame_id == "base_footprint") {
                        ROS_INFO("object %d dropped, change frame to map", i);
                        markers[i].header.frame_id = "map";
                        markers[i].header.stamp = ros::Time::now();
                        markers[i].pose.position.x = drop_x;
                        markers[i].pose.position.y = drop_y;
                        markers[i].color.a = 1.0f;
                        drop_x += 0.01;
                        ++i;
                    }
                }
                pickup_timer_trigger = false;
            }
        }
    }

    // check if near any unpicked object
    for (int i = 0; i < markers.size(); i++) {
        // skip things that have been picked up
        // both currently being carried and dropped off
        if (markers[i].header.frame_id == "base_footprint") {
            continue;
        }

        if (picked_up_status[i]) {
            continue;
        }

        // check if it has been so for enough time
        // (just passing by or actually picking it up)
        dee = calc_d(odom.pose.pose.position, pickups[i]);
        if (dee < th) {
            if (!pickup_timer_trigger) {
                pickup_timer_trigger = true;
                pickup_time = ros::Time::now();
            }
            else {
                double dt = (ros::Time::now() - pickup_time).toSec();
                if (dt > 0.5*t_wait) {
                    ROS_INFO("object %d picked up, change frame to robot", i);
                    picked_up_status[i] = true;
                    markers[i].header.frame_id = "base_footprint";
                    markers[i].pose.position.x = 0.25*cos(theta_carry);
                    markers[i].pose.position.y = 0.25*sin(theta_carry);
                    theta_carry += dtheta;
                    markers[i].header.stamp = ros::Time::now();
                    markers[i].color.a = 1.0f;
                    pickup_timer_trigger = false;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    // node setup
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;

    // read parameters
    ros::param::get("dropoff", dropoff);
    vector<double> pickups_flat;
    ros::param::get("pickups", pickups_flat);

    ROS_INFO("pickups");
    for (int i = 0; i + 2 <= pickups_flat.size(); i+=2) {
        vector<double> pickup {pickups_flat[i], pickups_flat[i+1]};
        pickups.push_back(pickup);
        ROS_INFO("%5.2f, %5.2f", pickup[0], pickup[1]);
    }

    // pubsub setup
    marker_pub = nh.advertise<visualization_msgs::Marker>("vis_marker", 1);
    ros::Subscriber odom_sub = nh.subscribe("amcl_pose", 1, odom_cb);

    // globals initialisation
    for (int i = 0; i < pickups.size(); i++) {
        picked_up_status.push_back(false);

        VisMarker marker_i;
        marker_i.header.frame_id = "map";
        marker_i.header.stamp = ros::Time::now();

        marker_i.ns = "pick_object";
        marker_i.id = i;

        marker_i.type = VisMarker::CUBE;
        marker_i.action = VisMarker::ADD;

        marker_i.pose.position.x = pickups[i][0];
        marker_i.pose.position.y = pickups[i][1];
        marker_i.pose.orientation.w = 1.0;

        marker_i.scale.x = 0.2;
        marker_i.scale.y = 0.2;
        marker_i.scale.z = 0.2;

        marker_i.color.r = 1.0f - (1.0*i/pickups.size());
        marker_i.color.g = 0.0f;
        marker_i.color.b = 0.0f + (1.0*i/pickups.size());
        marker_i.color.a = 1.0f;

        marker_i.lifetime = ros::Duration();

        markers.push_back(marker_i);
    }

    th = 0.1;

    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Got subsriber for the marker!");

    ros::Rate(1.0);
    // keep node alive
    while (ros::ok()) {
        ros::spinOnce();
        draw_markers();
    }

}
