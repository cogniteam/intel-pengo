/*
 * person_follower_node.cpp
 *
 *  Created on: May 13, 2019
 *      Author: Igor Makhtes <igor@cogniteam.com>
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <object_msgs/ObjectInBox.h>
#include <geometry_msgs/Twist.h>


using namespace std;


class PersonFollower {

public:

    PersonFollower()
        : target_(0, 0, 0) {

        ros::NodeHandle node;
        ros::NodeHandle nodePrivate("~");

        nodePrivate.param("tracking_timeout", trackingTimeout_, 0.5);
        nodePrivate.param("min_distance", minDistance_, 1.5);
        nodePrivate.param("max_distance", maxDistance_, 5.0);
        nodePrivate.param("max_speed", maxSpeed_, 110.0);
        nodePrivate.param("min_speed", minSpeed_, 110.0);

        nodePrivate.param("max_rotation", maxRotation_, 0.5);

        nodePrivate.param("steering_factor", steeringFactor_, 3.0);
        nodePrivate.param("enable", enable_, false);
        nodePrivate.param("base_frame", baseFrame_, string("camera_link"));
        nodePrivate.param("target", trackingTargetClassName_, string("person"));

        detectedObjectsSubscriber_ = node.subscribe("detected_objects", 1, 
                &PersonFollower::detectedObjectsCallback, this);

        enableSubscriber_ = node.subscribe("commands/person_follower/enable", 1, 
                &PersonFollower::enableCallback, this);

        commandPublisher_ = node.advertise<ackermann_msgs::AckermannDriveStamped>(
                "ackermann_cmd", 1, true);

        twistCommandPublisher_ = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, false);
                        
        statePublisher_ = node.advertise<std_msgs::Bool>(
                "events/person_follower/state", 1, true);
                        
        trackingStatePublisher_ = node.advertise<std_msgs::String>(
                "events/person_follower/tracking_state", 1, true);
                        
        targetPublisher_ = node.advertise<geometry_msgs::PoseStamped>(
                "debug/person_follower/target", 1, false);

        updateTimer_ = node.createTimer(ros::Rate(20), 
                &PersonFollower::updateTimerCallback, this);

        lastTargetUpdateTime_ = ros::Time::now() - ros::Duration(1000);

        publishState();

        updateTrackingState("lost");

    }

    virtual ~PersonFollower() {

    }

private:

    /**
     * @brief 
     * @param newState 
     */
    void updateTrackingState(string newState) {
        
        // Test if new state transition
        if (newState != trackingState_) {
            trackingState_ = newState;

            // Publish state update
            std_msgs::String msg;
            msg.data = newState;
            this->trackingStatePublisher_.publish(msg);
        }

    }

    void publishState() {
        std_msgs::Bool state;
        state.data = enable_;
        statePublisher_.publish(state);
    }

    void detectedObjectsCallback(const visualization_msgs::MarkerArray::Ptr& objects) {

        //
        // Closest person to previous detected person (if more than one detected)
        //
        tf::Vector3 closestPerson(-10000, 0, 0);
        bool personFound = false;

        static int detectionsInARow = 0;
        static int detectionsInARowNeeded = 3;

        for (auto&& object : objects->markers) {

            //
            // Person detected
            //
            
            if (object.text == trackingTargetClassName_) {

                tf::Vector3 poseVector(0, 0, 0);

                try {
                                        
                    poseVector = transformToBaseFrame(object.header.frame_id, 
                            ros::Time(0), object.pose.position);

                    //
                    // This person is closer to prev detected
                    //
                    if (target_.distance(closestPerson) > target_.distance(poseVector)) {
                        personFound = true;
                        detectionsInARow++;
                        closestPerson = poseVector;
                    }

                } catch(const std::exception& e) {
                    ROS_ERROR("Failed to transform to base frame %s", e.what());
                    continue;
                }
                                    
            }

        }

        //
        // Person found, continue following
        //
        if (personFound) {
            
            if (detectionsInARow >= detectionsInARowNeeded) {
                updateTarget(closestPerson);
            }

        } else {
            detectionsInARow = 0;
        }

    }

    void enableCallback(const std_msgs::Bool::Ptr& enable) {
        enable_ = enable->data;

        publishState();
    }

    tf::Vector3 transformToBaseFrame(const string sourceFrame, const ros::Time& stamp, 
            const geometry_msgs::Point& point) const {
        tfListener_.waitForTransform(baseFrame_, sourceFrame, stamp, ros::Duration(0.5));

        tf::Vector3 result;
        geometry_msgs::PointStamped pointSource;
        geometry_msgs::PointStamped pointTarget;

        pointSource.header.frame_id = sourceFrame;
        pointSource.header.stamp = stamp;
        pointSource.point = point;

        tfListener_.transformPoint(baseFrame_, pointSource, pointTarget);
        tf::pointMsgToTF(pointTarget.point, result);

        return result;
    }

    /**
     * @brief Target found
     * @param target 
     */
    void updateTarget(const tf::Vector3& target) {
        // ROS_INFO_THROTTLE(1.0, "Target updated [%f, %f]", target.x(), target.y());
        lastTargetUpdateTime_ = ros::Time::now();
        target_ = target;

        geometry_msgs::PoseStamped targetMsg;
        targetMsg.header.frame_id = baseFrame_;
        targetMsg.header.stamp = ros::Time::now();
        targetMsg.pose.position.x = target_.x();
        targetMsg.pose.position.y = target_.y();
        targetMsg.pose.orientation.w = 1.0;
        targetPublisher_.publish(targetMsg);
    }

    /**
     * @brief 
     */
    void updateTimerCallback(const ros::TimerEvent&) {

        //
        // Stop the robot if there were no detections for this amount of time
        //
        const ros::Duration detectionTimeout(trackingTimeout_);

        auto lastDetectionAge = ros::Time::now() - lastTargetUpdateTime_;
        
        if (lastDetectionAge > detectionTimeout) {
            
            // First time the target is lost - stop the robot and update 
            // tracking state
            if (trackingActive_) {
                ROS_WARN("Person lost, stop the aggression");
                updateTrackingState("lost");
                publishCommand(0, 0);
                trackingActive_ = false;
            } else {
                // Waiting for detections...
                ROS_INFO_THROTTLE(1.0, "Searching for a prey...");
            }

            return;

        } else {

            if (!trackingActive_) {
                trackingActive_ = true;
                updateTrackingState("tracking");
                ROS_WARN("Meatbag detected, attack it!");
            } else {
                ROS_INFO_THROTTLE(0.5, "Tracking...");
            }
        }

        //
        // Calculate velocity command
        //

        double distanceToTarget = target_.length();

        //
        // Compute speed 
        //

        // Ease in and out function (from min speed to max speed)

        // Range [0, 1]
        double targetDistanceRatio = (distanceToTarget - minDistance_) / (maxDistance_ - minDistance_);
        double speedRange = maxSpeed_ - minSpeed_;
        double easeFunction = -0.5 * (cos(M_PI * targetDistanceRatio) - 1);
        double speed = minSpeed_ + speedRange * easeFunction;
        double bearing = atan2(target_.y(), target_.x());


        //
        // Set speed to zero if target is not within min/max range but
        // still calculate bering to allow rotation in place to follow the
        // target
        //
        if (distanceToTarget < minDistance_) {
            // 
            // Person is too close
            //
            ROS_INFO_THROTTLE(0.5, "Prey is too close, wait for it to run away [%f]", 
                distanceToTarget);

            speed = 0.0;

        } else if (distanceToTarget > maxDistance_) {
            //
            // Person is too far
            //
            ROS_INFO_THROTTLE(0.5, "Prey is too far, lure it by intensive waiting [%f]", 
                distanceToTarget);

            speed = 0.0;
            
        } else {
            //
            // Distance is OK - following
            //
            ROS_INFO_THROTTLE(0.5, "Stalking [Distance = %f, Angle = %i]", distanceToTarget, 
                (int)angles::to_degrees(bearing));
        }

        // Used to normilize bearing estimation (bearing is clamped within range [-MAX_BEARING, +MAX_BEARING])
        const double MAX_BEARING = angles::from_degrees(50);

        // Clamp bearing estimation to range [-MAX, +MAX] in rads
        bearing = fmax(fmin(bearing, MAX_BEARING), -MAX_BEARING);

        // Translate bearing angle (in rads) to angular velocity 
        // within range [-MAX_ROTATION_VEL, +MAX_ROTATION_VEL]
        double angularVelocity = (bearing / MAX_BEARING) * maxRotation_;

        // Clamp rotation velocity to [-MAX_ROTATION_VEL, +MAX_ROTATION_VEL]
        angularVelocity = fmax(fmin(angularVelocity, maxRotation_), -maxRotation_);

        // Check if target in the center of the FOV (within +-5 degrees margin)
        bool targetInCenter = fabs(angles::to_degrees(bearing)) < 5.0;

        // Don't steer if target in center of FOV
        if (targetInCenter) {
            angularVelocity = 0;
        }

        // Tooo close, stop 
        if (distanceToTarget < 1.0) {
            angularVelocity = 0;
            speed = 0;
        }

        publishCommand(speed, angularVelocity);

    }

    void publishCommand(double speed, double steeringAngle) {

        if (!enable_) {
            return;
        }

        //
        // Ackermann model command publisher
        //
        ackermann_msgs::AckermannDriveStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.drive.speed = speed;
        cmd.drive.steering_angle = steeringAngle;
        commandPublisher_.publish(cmd);
        
        //
        // Diff drive model command publisher
        //
        geometry_msgs::Twist command;
        command.linear.x = speed;
        command.angular.z = steeringAngle;
        twistCommandPublisher_.publish(command);
    }

private:

    string baseFrame_ = "camera_link";

private:

    ros::Publisher commandPublisher_;

    ros::Publisher targetPublisher_;

    ros::Publisher statePublisher_;

    ros::Publisher trackingStatePublisher_;

    ros::Publisher twistCommandPublisher_;

    ros::Subscriber detectedObjectsSubscriber_;

    ros::Subscriber enableSubscriber_;

    tf::TransformListener tfListener_;

    ros::Time lastTargetUpdateTime_;

    tf::Vector3 target_;

    ros::Timer updateTimer_;

    bool trackingActive_ = false;

    bool zeroSpeed_;

    double minDistance_;

    double trackingTimeout_;

    double maxDistance_;

    double minSpeed_;

    double maxSpeed_;

    double maxRotation_;

    double steeringFactor_;

    bool enable_;

    /**
     * @brief Default is 'person'
     */
    std::string trackingTargetClassName_;

    /**
     * @brief 'lost' or 'tracking'
     */
    std::string trackingState_ = "";

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "person_follower_node");
    ros::NodeHandle node;
    PersonFollower follower;
    ros::spin();
    return 0;
}
