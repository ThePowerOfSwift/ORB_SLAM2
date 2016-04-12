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
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/PangolinViewer.h"
#include"../../../include/Viewer.h"
#include "MapPublisher.hpp"
#include "Camera.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
  void SaveMapCallback(const std_msgs::String::ConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ROS_ORB_SLAM::MapPublisher _map_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
 	bool use_pango_viewer;
	cerr << "## ORB ROS parameters:" << endl;
    ros::param::param<bool>("orb_use_pango_viewer", use_pango_viewer, true);
    cerr << "# orb use pango viewer " << use_pango_viewer << endl;
	
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::Viewer *viewer;
	if (use_pango_viewer)
	  viewer = new ORB_SLAM2::PangolinViewer(argv[2]);
	else
	  viewer = new ORB_SLAM2::Viewer();

    cerr << "# load voc"<<endl;
	ORB_SLAM2::ORBVocabulary voc;
	voc.loadFromBinaryFile(argv[1]);
    cerr << "# load cam"<<endl;
	ORB_SLAM2::Camera::Load(argv[2]);
	ORB_SLAM2::Map map;
    cerr << "# load map"<<endl;
	map.Load("/home/mshomin/Downloads/ORB_SLAM2/Examples/ROS/ORB_SLAM2/test.map", voc);

    cerr << "# init system"<<endl;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, viewer, &map, &voc);
	//SLAM.ActivateLocalizationMode();

	//ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, viewer);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

	ros::Subscriber sub2 = nh.subscribe("/ORB_SLAM2/save_map", 1000, &ImageGrabber::SaveMapCallback, &igb);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    if (use_pango_viewer) {
      ros::spin();
    }
    else {
      int fps = 30;
      ros::Rate r(fps);
      while (ros::ok()) {
	//igb._frame_pub.refresh();
	ros::spinOnce();
	r.sleep();
      }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


void ImageGrabber::SaveMapCallback(const std_msgs::String::ConstPtr& msg) {
  cerr << "Saving to " << msg->data.c_str() << endl;
  mpSLAM->SaveMap(msg->data.c_str());
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cout << cv_ptrD->image.rows << " x " << cv_ptrD->image.cols << ", type: " << cv_ptrD->image.type() << endl;
    /*cv::imshow("rgb",cv_ptrRGB->image);
    cv::imshow("depth",cv_ptrD->image);
    cv::waitKey(20);*/
    cv::Mat cam_pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
    //_map_pub.PublishCurrentCamera(cam_pose);
}


