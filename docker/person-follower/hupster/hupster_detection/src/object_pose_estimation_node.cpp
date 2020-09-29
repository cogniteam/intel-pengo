/*
 * object_pose_estimation_node.cpp
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


#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <depth_image_proc/depth_conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <visualization_msgs/MarkerArray.h>


class ObjectPoseEstimator {

public:

    ObjectPoseEstimator() {
        ros::NodeHandle node;

        image_transport::ImageTransport it(node);

        depthSubscriber_ = it.subscribe("camera/aligned_depth_to_color/image_raw", 1,
                &ObjectPoseEstimator::depthCallback, this);

        objectSubscriber_ = node.subscribe("openvino_toolkit/detected_objects", 1,
                &ObjectPoseEstimator::detectedObjectsCallback, this);

        cameraInfoSubscriber_ = node.subscribe("camera/color/camera_info", 1,
                &ObjectPoseEstimator::cameraInfoCallback, this);

        markerPublisher_ = node.advertise<visualization_msgs::MarkerArray>(
                "detected_objects", 10, true);
        
    }

    virtual ~ObjectPoseEstimator() {

    }

private:

    void cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr& cameraInfo) {
        cameraModel_.fromCameraInfo(*cameraInfo);
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& depthImage) {

        depthImage_ = cv_bridge::toCvShare(depthImage)->image;

        // double value = depthMat.at<uint16_t>(
        //         cv::Point(depthMat.cols / 2, depthMat.rows / 2));

        // ROS_INFO("Depth received %f", value);

        // cv::imshow("view", depthMat);
        // cv::waitKey(30);
    }

    void detectedObjectsCallback(const object_msgs::ObjectsInBoxes::Ptr& objects) {

        if (!cameraModel_.initialized() || depthImage_.cols == 0 || depthImage_.rows == 0) {
            return;
        }

        cv::Rect imageRect(0, 0, depthImage_.cols, depthImage_.rows);
        visualization_msgs::MarkerArray markers;

        for (auto&& object : objects->objects_vector) {

            int pointsCount = 0;
            cv::Point3d points3dSum(0, 0, 0);

            int OFFSET_X = object.roi.width * 0.4;
            int OFFSET_Y = object.roi.height * 0.4;

            for (size_t y = object.roi.y_offset + OFFSET_Y; y < object.roi.y_offset + object.roi.height - OFFSET_Y; y++) {
                for (size_t x = object.roi.x_offset + OFFSET_X; x < object.roi.x_offset + object.roi.width - OFFSET_X; x++) {
                    cv::Point2d uv(x, y);

                    if (!imageRect.contains(uv)) {
                        continue;
                    }

                    auto point3d = cameraModel_.projectPixelTo3dRay(uv);
                    auto depth = depthImage_.at<uint16_t>(uv);

                    if (depth < 0.001) {
                        continue;
                    }

                    point3d.z = depth / 1000.0;

                    points3dSum += point3d;
                    pointsCount++;

                }    
            }
            

            if (pointsCount > 0) {

                auto centerOfMass = points3dSum / pointsCount;

                cv::Point2d centerPixel(object.roi.x_offset + object.roi.width / 2, 
                        object.roi.y_offset + object.roi.height / 2);
                auto objectRay = cameraModel_.projectPixelTo3dRay(centerPixel);
                double bearing = atan2(objectRay.z, -objectRay.x);
                double distance = cv::norm(centerOfMass) - 0.6;

                tf::Vector3 objectVector(distance * sin(bearing), 
                        distance * cos(bearing), 0);

                ROS_INFO("%s at [%f, %f, %f]", object.object.object_name.c_str(), 
                        centerOfMass.x, centerOfMass.y, centerOfMass.z);

                visualization_msgs::Marker marker;
                marker.lifetime = ros::Duration(0.2);
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.header.frame_id = cameraModel_.tfFrame();
                marker.header.stamp = objects->header.stamp;
                marker.id = rand();
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = -objectVector.y();
                marker.pose.position.y = 0;
                marker.pose.position.z = objectVector.x();
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.text = object.object.object_name;

                markers.markers.push_back(marker);
            }

        }

        markerPublisher_.publish(markers);

    }

private:

    image_transport::Subscriber depthSubscriber_;

    ros::Subscriber objectSubscriber_;

    ros::Subscriber cameraInfoSubscriber_;

    ros::Publisher markerPublisher_;

    image_geometry::PinholeCameraModel cameraModel_;

    cv::Mat depthImage_;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "hupster_object_pose_estimation_node");
    ros::NodeHandle node;
    ObjectPoseEstimator estimator;
    ros::spin();
    return 0;
}
