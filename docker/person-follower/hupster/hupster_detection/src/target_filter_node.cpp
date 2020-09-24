#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


/**
 * @brief Takes daetected objects with pose and returns specified object
 * 
 */

class ObjectsFilter
{
public:
    ObjectsFilter() {
        ros::NodeHandle node;

        detectedObjectsSub_ = node.subscribe("detected_objects", 1, &ObjectsFilter::callback, this);
        
        filteredPub_ = node.advertise<geometry_msgs::PoseStamped>("detected_objects_filtered", 1);

        ros::NodeHandle nodePrivate("~");

        nodePrivate.param<std::string>("target", target_, "person");
        nodePrivate.param<std::string>("odom_frame", odomFrame_, "odom");

    }

    virtual ~ObjectsFilter() {

    }

private:

    void callback (const visualization_msgs::MarkerArrayConstPtr& msg) {
        
        for (auto m : msg->markers) {
            
            tfListener_.waitForTransform(odomFrame_, m.header.frame_id, m.header.stamp, ros::Duration(10));

            geometry_msgs::PoseStamped sPose;
            geometry_msgs::PoseStamped sPoseConv;

            sPose.header = m.header;
            sPose.pose = m.pose;

            tfListener_.transformPose(odomFrame_, sPose, sPoseConv);
            sPoseConv.pose.position.z = 0;
            sPoseConv.pose.orientation.x = 0;
            sPoseConv.pose.orientation.y = 0;
            sPoseConv.pose.orientation.z = 0;
            sPoseConv.pose.orientation.w = 1;

            filteredPub_.publish(sPoseConv);
        }
        
    }

private:

    ros::Subscriber detectedObjectsSub_;

    ros::Publisher filteredPub_;

    std::string target_;

    std::string odomFrame_;

    tf::TransformListener tfListener_;

    visualization_msgs::MarkerArray objectMessage_;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hupster_object_pose_filter_node");
    ros::NodeHandle node;
    ObjectsFilter filter;
    ros::spin();
    return 0;
}