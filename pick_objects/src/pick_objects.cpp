#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClt;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_things_up");

    // get parameters
    vector<double> dropoff;
    vector<double> pickups_flat;
    vector< vector<double> > pickups;
    double t_wait;

    ros::param::get("dropoff", dropoff);
    ros::param::get("pickups", pickups_flat);
    ros::param::get("t_wait", t_wait);

    ROS_INFO("dropoff: (%4.2f, %4.2f)", dropoff[0], dropoff[1]);

    for (int i = 0; i < pickups_flat.size(); i+=2) {
        vector<double> pickup {pickups_flat[i], pickups_flat[i+1]};
        pickups.push_back(pickup);
    }

    ROS_INFO("pickups:");
    for (int i = 0; i < pickups.size(); i++) {
        ROS_INFO("(%4.2f, %4.2f)", pickups[i][0], pickups[i][1]);
    }

//    ROS_INFO("end of test");
//    return 0;

    MoveBaseClt ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";

    // go to each pickup locations
    for (int i = 0; i < pickups.size(); i++) {
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = pickups[i][0];
        goal.target_pose.pose.position.y = pickups[i][1];
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal %d: (%4.2f, %4.2f)", i, pickups[i][0], pickups[i][1]);
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully reached goal");
            ros::Duration(t_wait).sleep();
        }
        else {
            ROS_INFO("Unable to reach goal. Abort.");
            break; // abort to dropoff
        }
    }

    // go to dropoff
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = dropoff[0];
    goal.target_pose.pose.position.y = dropoff[1];
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal to dropoff: (%4.2f, %4.2f)", dropoff[0], dropoff[1]);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successfully reached dropoff point");
    }
    else {
        ROS_INFO("Unable to reach dropoff");
    }

    // keep node alive
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
