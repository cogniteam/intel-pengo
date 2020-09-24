#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/String.h>


/**
 * @brief Takes daetected objects with pose and returns specified object
 * 
 */

class PersonFollower
{
public:

    PersonFollower() {

        ros::NodeHandle node;

        detectedObjectsSub_ = node.subscribe("detected_objects", 1, &PersonFollower::callback, this);
        
        targetPub_ = node.advertise<geometry_msgs::PoseStamped>("person_follower/target", 1);

        statePublisher_ = node.advertise<std_msgs::String>("person_follower/state", 1, true);

        ros::NodeHandle nodePrivate("~");

        double rate;

        nodePrivate.param<std::string>("target", target_, "person");
        nodePrivate.param<std::string>("odom_frame", odomFrame_, "odom");
        nodePrivate.param("timeout", timeout_, 2.0);
        nodePrivate.param("rate", rate, 5.0);

        timer_ = node.createTimer(ros::Rate(rate), &PersonFollower::timerCallback, this);

        setState("lost");

    }
    virtual ~PersonFollower() {
    }

private:

    void setState(const std::string& newState) {
        if (newState != state_)
        {
            std_msgs::String stateMsg;
            stateMsg.data = newState;
            statePublisher_.publish(stateMsg);
            state_ = newState;

        }
    }

    void timerCallback (const ros::TimerEvent&) {
        // if (state_ != "tracking") {
        //     return;
        // }

        auto currentTime = ros::Time::now();
        if ((currentTime - lastObjectTimeUpdate_).toSec() > timeout_) {
            geometry_msgs::PoseStamped sPose;
            sPose.header.frame_id = "base_link";
            sPose.header.stamp = ros::Time(0);
            sPose.pose.orientation.x = 0;
            sPose.pose.orientation.y = 0;
            sPose.pose.orientation.z = 0;
            sPose.pose.orientation.w = 1;

            sPose.pose.position.x = 0; 
            sPose.pose.position.y = 0;
            sPose.pose.position.z = 0;

            targetPub_.publish(sPose);

            setState("lost");
            return;
        }

        lastGoal_.pose.orientation = calculateYaw(lastGoal_);

        targetPub_.publish(lastGoal_);
    }

    void callback (const visualization_msgs::MarkerArrayConstPtr& msg) {

        for (auto m : msg->markers) {
            if(m.text == target_)
            {
                tfListenerGoalPose_.waitForTransform(
                    odomFrame_, m.header.frame_id, m.header.stamp, ros::Duration(10));

                geometry_msgs::PoseStamped sPose;
                geometry_msgs::PoseStamped sPoseConv;

                sPose.header = m.header;
                sPose.pose = m.pose;

                tfListenerGoalPose_.transformPose(odomFrame_, sPose, sPoseConv);
                sPoseConv.pose.position.z = 0;
                sPoseConv.pose.orientation.x = 0;
                sPoseConv.pose.orientation.y = 0;
                sPoseConv.pose.orientation.z = 0;
                sPoseConv.pose.orientation.w = 1;

                lastGoal_ = sPoseConv;
                
                if (state_ != "tracking") {
                    targetPub_.publish(lastGoal_);
                }

                setState("tracking");

                
                lastObjectTimeUpdate_ = ros::Time::now();
            } 
        }
    }

    geometry_msgs::Quaternion calculateYaw(geometry_msgs::PoseStamped goalPose) {

        tfListenerRobotPose_.waitForTransform(
            odomFrame_, "base_link", ros::Time(0), ros::Duration(10));

        tf::StampedTransform tfStamped;

        tfListenerRobotPose_.lookupTransform(odomFrame_, "base_link", ros::Time(0), tfStamped);

        double deltaX = goalPose.pose.position.x - tfStamped.getOrigin().getX();
        double deltaY = goalPose.pose.position.y - tfStamped.getOrigin().getY();

        return tf::createQuaternionMsgFromYaw(atan2(deltaY,deltaX));
    }

private:

    ros::Subscriber detectedObjectsSub_;

    ros::Publisher targetPub_;

    ros::Publisher statePublisher_;

    std::string target_;

    std::string odomFrame_;

    double timeout_;;

    tf::TransformListener tfListenerGoalPose_;

    tf::TransformListener tfListenerRobotPose_;

    ros::Time lastObjectTimeUpdate_;

    ros::Timer timer_;

    std::string state_ = "";

    geometry_msgs::PoseStamped lastGoal_;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_follower_node");
    ros::NodeHandle node;
    PersonFollower filter;
    ros::spin();
    return 0;
}