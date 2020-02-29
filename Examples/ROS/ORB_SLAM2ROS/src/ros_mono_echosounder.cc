/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include"std_msgs/String.h"
#include"ping_nodelet/Ping.h"

#include<opencv2/core/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include"../../../include/System.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ping_nodelet::Ping> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;

class ImageDistanceGrabber
{
public:
    ImageDistanceGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle* nodeHandler):mpSLAM(pSLAM){
        this->nh = *nodeHandler;

        this->img_sub.subscribe(nh, "/camera/image_raw", 5);
        this->echosounder_sub.subscribe(nh, "/ping_nodelet/ping", 5);

        this->sync.reset(new Sync(SyncPolicy(15), this->img_sub, this->echosounder_sub));
        this->sync->registerCallback(boost::bind(&ImageDistanceGrabber::GrabImageAndDistance, this, _1, _2));
    }

    void GrabImageAndDistance(const sensor_msgs::ImageConstPtr& image_msg, const ping_nodelet::Ping::ConstPtr& echosounder_msg);

    ORB_SLAM2::System* mpSLAM;
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    message_filters::Subscriber<ping_nodelet::Ping> echosounder_sub;

    boost::shared_ptr<Sync> sync;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nodeHandler;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, &nodeHandler,true);

    ImageDistanceGrabber idgb(&SLAM, &nodeHandler);

    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageDistanceGrabber::GrabImageAndDistance(const sensor_msgs::ImageConstPtr& image_msg, const ping_nodelet::Ping::ConstPtr& echosounder_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cv::Mat cur_image;

    //cv::rotate(cv_ptr->image, cur_image, cv::ROTATE_90_CLOCKWISE);

    mpSLAM->IntegrateEchosounder(echosounder_msg->distance, echosounder_msg->confidence);
    //mpSLAM->TrackMonocular(cur_image, cv_ptr->header.stamp.toSec());
    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}
