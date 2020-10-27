/**
 * @brief 
 * 
 * @file PengoStalkerRos.h
 * 
 * @author Igor Makhtes (igor@cogniteam.com)
 * @date 2020-10-13
 * @copyright Cogniteam (c) 2020
 * 
 * Cogniteam LTD
 *   
 * Unpublished Copyright (c) 2016-2020 Cogniteam
 *    
 * 
 */

#ifndef EDF8BC6D_BE66_4974_B3A7_482FA54ED7EA
#define EDF8BC6D_BE66_4974_B3A7_482FA54ED7EA

#include <thread>
#include <future>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <std_srvs/Empty.h>


namespace pengo {


/**
 * @brief Stalking mission state
 */
enum class MissionState {

    /**
     * @brief Doing nothing
     */
    Idle,

    /**
     * @brief Driving patrol path repeatedly while waiting for abduction or 
     * person
     */
    Patroling,

    /**
     * @brief Robot was abducted (picked up), records traveled path using 
     * T265 visual odometry
     */
    Abducted,

    /**
     * @brief Folloing a person who has been detected by OpenVino (using 
     * front camera)
     */
    Stalking,

    /**
     * @brief Driving back to last patrol point by following recorded from
     * visual odometry
     */
    Backtracking,
};


/**
 * @brief Pengo wheels state
 */
struct WheelsState {
    bool leftOnGround = true;
    bool rightOnGround = true;
};


/**
 * @brief 
 */
class PengoStalkerRos
{

public:

    /**
     * @brief Construct a new Pengo Stalker Ros object
     */
    PengoStalkerRos();

    /**
     * @brief Destroys the Pengo Stalker Ros object
     */
    ~PengoStalkerRos();

public:

    /**
     * @brief Start patroling around while searching for people to follow.
     * In case of theft attempt, will backtrack to last point on patrol route 
     * using path recorded by T265 camera.
     */
    void startPatrol();

    /**
     * @brief Stops all activity immediately
     */
    void stopPatrol();

private:

    /**
     * @brief Generates patrol path consisted of {pointsCount} poses around
     * current robot location
     * @param pointsCount Number of points in patrol path
     * @param radius Distance of patrol points from current position
     */
    void generatePatrolRoute(int pointsCount, double radius);

    /**
     * @brief Publishes current patrol (using ROS action) point as 
     * navigation goal to move_base
     */
    void startNavigationToNextPatrolPoint();

    /**
     * @brief 
     */
    void startNavigationToNextRecordedPathPoint();

    /**
     * @brief Updates mission state
     * @param newState New mission state
     */
    void updateState(MissionState newState);

    /**
     * @brief Clears move_base's costmaps using ROS service call. 
     * This method used to clear obstacles map which can block path planning
     */
    void clearCostmaps();

    /**
     * @brief New visual odometry measurement from RealSense T265 camera
     * @param odom 
     */
    void visualOdometryCallback(const nav_msgs::Odometry::Ptr& odom);

    /**
     * @brief Robot platform odometry
     * @param odom 
     */
    void odometryCallback(const nav_msgs::Odometry::Ptr& odom);

    /**
     * @brief Wheel drop sensors event callback
     * @param event 
     */
    void wheelDropEventCallback(const kobuki_msgs::WheelDropEvent::Ptr& event);

    /**
     * @brief Person follower state change callback ('lost' or 'tracking')
     */
    void personFollowerStateCallback(
            const std_msgs::String::Ptr& state);

    /**
     * @brief Sets the Person Follower Enabled/Disabled state
     * @param enabled 
     */
    void setPersonFollowerEnabled(bool enabled);

    /**
     * @brief 
     * @param state 
     * @param result 
     */
    void navigationGoalReached(
            const actionlib::SimpleClientGoalState & state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result);

    /**
     * @brief Start/Stop command received
     */
    void enableCallback(const std_msgs::Bool);

private:

    /**
     * @brief Start/Stop command subscriber
     */
    ros::Subscriber enableSubscriber_;

    /**
     * @brief Wheel drop event ROS subscriber
     */
    ros::Subscriber wheelDropEventSubscriber_;
    
    /**
     * @brief Start/stops person follower node
     */
    ros::Publisher personFollowEnablePublisher_;

    /**
     * @brief Patrol path publisher
     */
    ros::Publisher patrolPathPublisher_;

    /**
     * @brief Recorded by from visual odometry publisher
     */
    ros::Publisher recordedPathPublisher_;

    /**
     * @brief Robot platform odometry, used to calculate patrol route in odom
     * frame
     */
    ros::Subscriber odomSubscriber_;

    /**
     * @brief 'tracking' or 'lost' from person follower node
     */
    ros::Subscriber personFollowerStateSubscriber_;

    /**
     * @brief 
     */
    ros::Subscriber visualOdomSubscriber_;

    /**
     * @brief move_base clear costmaps
     */
    ros::ServiceClient clearCostmapsService_;

    /**
     * @brief 
     */
    ros::Timer updateTimer_;

    /**
     * @brief Last wheel drop event
     */
    kobuki_msgs::WheelDropEvent::Ptr lastWheelDropEvent_;

    /**
     * @brief Path recorded from T265 visual odometry
     */
    nav_msgs::Path::Ptr visualOdometryPath_;

    /**
     * @brief Patrol points
     */
    nav_msgs::Path::Ptr patrolPath_;

    /**
     * @brief Index of current patrol point
     */
    int currentPatrolPointIndex_;

    /**
     * @brief Mission state, initially idle
     */
    MissionState missionState_;

    /**
     * @brief Move base navigation ROS action client
     */
    boost::shared_ptr<actionlib::SimpleActionClient<
            move_base_msgs::MoveBaseAction>> moveBaseActionClient_;

    /**
     * @brief Robot's pose calculated using wheel odometry
     */
    tf::Transform odomPoseTf_;

    /**
     * @brief 
     */
    WheelsState wheelsState_;

    /**
     * @brief ROS param - number of patrol points to generate
     */
    int patrolPoints_;

    /**
     * @brief ROS param - distance of patrol points from current robot's pose
     */
    double patrolRadius_;

};


} /* namespace pengo */


#endif /* EDF8BC6D_BE66_4974_B3A7_482FA54ED7EA */
