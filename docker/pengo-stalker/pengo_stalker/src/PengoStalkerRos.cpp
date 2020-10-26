/**
 * @brief 
 * 
 * @file PengoStalkerRos.cpp
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


#include <pengo_stalker/PengoStalkerRos.h>


namespace pengo {


/**
 * @brief Construct a new Pengo Stalker Ros object
 */
PengoStalkerRos::PengoStalkerRos() {

    // Init all
    missionState_ = MissionState::Idle;
    currentPatrolPointIndex_= 0;
    patrolPath_.reset(new nav_msgs::Path());
    visualOdometryPath_.reset(new nav_msgs::Path());
    odomPoseTf_.setIdentity();

    ros::NodeHandle node;

    moveBaseActionClient_.reset(
            new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
                    "move_base", true));

    odomSubscriber_ = node.subscribe("odom", 10, 
            &PengoStalkerRos::odometryCallback, this);

    visualOdomSubscriber_ = node.subscribe("camera/odom", 10, 
            &PengoStalkerRos::visualOdometryCallback, this);

    wheelDropEventSubscriber_ = node.subscribe("mobile_base/events/wheel_drop", 
            10, &PengoStalkerRos::wheelDropEventCallback, this);

    enableSubscriber_ = node.subscribe("commands/pengo_stalker/enable", 
            1, &PengoStalkerRos::enableCallback, this);
    
    personFollowEnablePublisher_ = node.advertise<std_msgs::Bool>(
            "commands/person_follower/enable", 1, true);
}

/**
 * @brief Destroys the Pengo Stalker Ros object
 */
PengoStalkerRos::~PengoStalkerRos() {

}

/**
 * @brief Start patroling around while searching for people to follow.
 * In case of theft attempt, will backtrack to last point on patrol route 
 * using path recorded by T265 camera.
 */
void PengoStalkerRos::startPatrol() {

    setPersonFollowerEnabled(false);

    this->updateState(MissionState::Patroling);

    // First generate patrol route
    generatePatrolRoute(4, 0.5);

    // Reset current point index
    currentPatrolPointIndex_ = 0;

    // Send first goal
    startNavigationToNextPatrolPoint();
}

/**
 * @brief Stops all activity immediately
 */
void PengoStalkerRos::stopPatrol() {

    // Stop navigation
    moveBaseActionClient_->cancelAllGoals();

    this->updateState(MissionState::Idle);
}

/**
 * @brief Generates patrol path consisted of {pointsCount} poses around
 * current robot location
 * @param pointsCount Number of points in patrol path
 * @param radius Distance of patrol points from current position
 */
void PengoStalkerRos::generatePatrolRoute(int pointsCount, double radius) {

    // Reset path
    this->patrolPath_.reset(new nav_msgs::Path());

    this->patrolPath_->header.frame_id = "odom";
    this->patrolPath_->header.stamp = ros::Time::now();

    ROS_INFO("Generating patrol path:");

    for (int i = 0; i < pointsCount; i++) {
        double angleStep = ((M_PI * 2) / pointsCount);


        tf::Vector3 point(
                radius * cos(angleStep * i),
                radius * sin(angleStep * i), 
                0);

        double angle = atan2(point.y(), point.x());

        tf::Transform offset(
                tf::createQuaternionFromYaw(angle), 
                point);

        auto newPoseTf = odomPoseTf_ * offset;

        geometry_msgs::PoseStamped newPoseMsg;
        newPoseMsg.header = patrolPath_->header;
        tf::poseTFToMsg(newPoseTf, newPoseMsg.pose);        

        this->patrolPath_->poses.push_back(newPoseMsg);

        ROS_INFO(" - %i point [%f, %f]", i, newPoseMsg.pose.position.x, 
                newPoseMsg.pose.position.y);

    }

    ROS_INFO("Patrol route created");
    
}

/**
 * @brief Publishes current patrol (using ROS action) point as 
 * navigation goal to move_base
 */
void PengoStalkerRos::startNavigationToNextPatrolPoint() {
    move_base_msgs::MoveBaseGoal goal;

    if (this->patrolPath_->poses.size() == 0) {
        ROS_ERROR("Can't start patrol, path is empty");
        return;
    }

    // Loop 
    currentPatrolPointIndex_ %= patrolPath_->poses.size();

    goal.target_pose = patrolPath_->poses[currentPatrolPointIndex_];
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Going to patrol point #%i", currentPatrolPointIndex_);

    // Increment to next waypoint
    currentPatrolPointIndex_++;
    currentPatrolPointIndex_ %= patrolPath_->poses.size();

    // Start move_base
    this->moveBaseActionClient_->sendGoal(
            goal, 
            boost::bind(&PengoStalkerRos::navigationGoalReached, this, _1, _2));
    
}

/**
 * @brief Publishes current patrol (using ROS action) point as 
 * navigation goal to move_base
 */
void PengoStalkerRos::startNavigationToNextRecordedPathPoint() {
    move_base_msgs::MoveBaseGoal goal;

    if (this->visualOdometryPath_->poses.size() == 0) {
        ROS_ERROR("Path finished, shouldn't be called");
        return;
    }

    goal.target_pose = visualOdometryPath_->poses[
            visualOdometryPath_->poses.size() - 1];

    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Going back on recorded path #%i", 
            (int)(visualOdometryPath_->poses.size() - 1));

    // Start move_base
    this->moveBaseActionClient_->sendGoal(
            goal, 
            boost::bind(&PengoStalkerRos::navigationGoalReached, this, _1, _2));

    
}

template<typename T>
void pop_front(std::vector<T>& vec)
{

    if (vec.size() == 0) {
        return;
    }

    vec.front() = std::move(vec.back());
    vec.pop_back();
}

/**
 * @brief 
 * @param state 
 * @param result 
 */
void PengoStalkerRos::navigationGoalReached(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResult::ConstPtr& result) {
  

    if (this->missionState_ == MissionState::Patroling) {
        ROS_INFO("Patrol point reached");
        // Run in thread
        std::future<void> fut = std::async(std::launch::async, [&]() {
            startNavigationToNextPatrolPoint();
        });

    } else if (this->missionState_ == MissionState::Backtracking) {
        ROS_INFO("Backtrack point reached");

        // Erase first point, and move to next one
        pop_front(visualOdometryPath_->poses);

        // Check finish condition
        if (visualOdometryPath_->poses.size() == 0) {
            // Done
            ROS_INFO("Backtracking finished, continuing patrol...");
            updateState(MissionState::Patroling);
            return;
        }

        // Run in thread
        std::future<void> fut = std::async(std::launch::async, [&]() {
            startNavigationToNextRecordedPathPoint();
        });

    } else {
        ROS_INFO("Navigation finished outside Patroling mission state");
    }
    
}

/**
 * @brief Updates mission state
 * @param newState New mission state
 */
void PengoStalkerRos::updateState(MissionState newState) {

    if (missionState_ != newState) {
        missionState_ = newState;

        switch (newState)
        {
            case MissionState::Patroling:

                break;
            case MissionState::Abducted:
            case MissionState::Stalking: {
                
                // We didn't reach the goal so decrease index to repeat the same
                // goal on return
                currentPatrolPointIndex_--;

                // Start recording new path
                this->setPersonFollowerEnabled(true);
                visualOdometryPath_.reset(new nav_msgs::Path());
                break;
            }
            case MissionState::Backtracking: {
                // Stop in any case
                this->setPersonFollowerEnabled(false);
                
                // Clear costmaps
                
                clearCostmaps();
                
                // Reverse path
                
                std::reverse(visualOdometryPath_->poses.begin(), 
                        visualOdometryPath_->poses.end());

                // Navigate
                // Send goal
                this->startNavigationToNextRecordedPathPoint();    

                break;
            }
        }

    }

}

/**
 * @brief Clears move_base's costmaps using ROS service call. 
 * This method used to clear obstacles map which can block path planning
 */
void PengoStalkerRos::clearCostmaps() {

}

/**
 * @brief New visual odometry measurement from RealSense T265 camera
 * @param odom 
 */
void PengoStalkerRos::visualOdometryCallback(
        const nav_msgs::Odometry::Ptr& odom) {

    //
    // In both states record the traveled path using VO
    //
    if (this->missionState_ == MissionState::Abducted ||
            this->missionState_ == MissionState::Stalking) {
        //
        // Record path 
        //

        if (!visualOdometryPath_) {
            visualOdometryPath_.reset(new nav_msgs::Path());
        }

        if (visualOdometryPath_->poses.size() == 0) {
            // First odom, just add it to path
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = "odom";
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.pose = odom->pose.pose;
            visualOdometryPath_->poses.push_back(poseStamped);
            return;
        }

        tf::Vector3 lastOdomPoint;
        tf::pointMsgToTF(visualOdometryPath_->poses[
                visualOdometryPath_->poses.size() - 1]
                        .pose.position, lastOdomPoint);

        tf::Vector3 newOdomPoint;
        tf::pointMsgToTF(odom->pose.pose.position, newOdomPoint);

        //
        // 
        //
        if (lastOdomPoint.distance(newOdomPoint) > 0.2) {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = "odom";
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.pose = odom->pose.pose;
            visualOdometryPath_->poses.push_back(poseStamped);
        }

    }
}

/**
 * @brief New robot wheel odometry measurement 
 * @param odom 
 */
void PengoStalkerRos::odometryCallback(const nav_msgs::Odometry::Ptr& odom) {
    this->odomPoseTf_.setOrigin(tf::Vector3(
        odom->pose.pose.position.x,
        odom->pose.pose.position.y,
        odom->pose.pose.position.z
    ));

    this->odomPoseTf_.setRotation(tf::Quaternion(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    ));
}

/**
 * @brief Wheel drop sensors event callback
 * @param event 
 */
void PengoStalkerRos::wheelDropEventCallback(
        const kobuki_msgs::WheelDropEvent::Ptr& event) {
    if (event->wheel == event->LEFT) {
        wheelsState_.leftOnGround = (event->state == event->RAISED);
    } else {
        // Right
        wheelsState_.rightOnGround = (event->state == event->RAISED);
    }

    if (missionState_ == MissionState::Patroling) {

        if (!wheelsState_.leftOnGround || !wheelsState_.rightOnGround) {
            // Abduction detected!
            ROS_INFO("abducted!");
            updateState(MissionState::Abducted);
        }

    }

    if (missionState_ == MissionState::Abducted) {
        
        if (wheelsState_.leftOnGround && wheelsState_.rightOnGround) {
            // Back on ground! Retreat!
            ROS_INFO("Back on the ground!");
            updateState(MissionState::Backtracking);
        }

    }

}

/**
 * @brief Person follower state change callback ('lost' or 'tracking')
 */
void PengoStalkerRos::personFollowerStateCallback(
        const std_msgs::String::Ptr& state) {
    
    if (this->missionState_ == MissionState::Patroling &&
            state->data == "tracking") {
                
        // Prey detected, attack it!
        updateState(MissionState::Stalking);

    } else if (this->missionState_ == MissionState::Stalking) {

        if (state->data == "lost") {
            // Prey lost, backtrack
            updateState(MissionState::Backtracking);
        }

    }
}

/**
 * @brief Sets the Person Follower Enabled/Disabled state
 * @param enabled 
 */
void PengoStalkerRos::setPersonFollowerEnabled(bool enabled) {
    std_msgs::Bool msg;
    msg.data = enabled;
    personFollowEnablePublisher_.publish(msg);
}

void PengoStalkerRos::enableCallback(const std_msgs::Bool enable) {

    if (enable.data) {
        if (missionState_ == MissionState::Idle) {
            startPatrol();
        }
    } else {
        if (missionState_ != MissionState::Idle) {
            stopPatrol();
        }
    }


}


} /* namespace pengo */
