/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "OCVImageStreamThread.h"
// #include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>

#include <sys/time.h>
double currentTime(){
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms/1000.;
}

namespace lsd_slam
{


using namespace cv;

OCVImageStreamThread::OCVImageStreamThread():
  capture_(cv::VideoCapture("seq38.avi"))
{
	// subscribe
	// vid_channel = nh_.resolveName("image");
	// vid_sub          = nh_.subscribe(vid_channel,1, &OCVImageStreamThread::vidCb, this);

	// wait for cam calib
	width_ = height_ = 0;

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;
	lastSEQ = 0;

	haveCalib = false;
}

OCVImageStreamThread::~OCVImageStreamThread()
{
	delete imageBuffer;
}

void OCVImageStreamThread::setCalibration(std::string file)
{
	if(file == "")
	{
		// ros::Subscriber info_sub         = nh_.subscribe(nh_.resolveName("camera_info"),1, &OCVImageStreamThread::infoCb, this);
    infoCb(capture_);

		printf("WAITING for ROS camera calibration!\n");
		// while(width_ == 0)
		{
			// ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		// info_sub.shutdown();
	}
	else
	{
		undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(undistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(2, 0);
		cy_ = undistorter->getK().at<double>(2, 1);

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight();
	}

	haveCalib = true;
}

void OCVImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void OCVImageStreamThread::operator()()
{
	// ros::spin();
  while (1){
    fprintf(stderr,"+");
    cv::Mat frame;
    capture_ >> frame;
    vidCb(frame);
    usleep(30*1000);
  }
    
	exit(0);
}


void OCVImageStreamThread::vidCb(const cv::Mat img)
{
	if(!haveCalib) {printf("not calibrated yet!\n"); return;}

	// cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	// if(img->header.seq < (unsigned int)lastSEQ)
	// {
	// 	printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
	// 	lastSEQ = 0;
	// 	return;
	// }
	// lastSEQ = img->header.seq;

	TimestampedMat bufferItem;
	// if(img->header.stamp.toSec() != 0)
	// 	bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
	// else
  // TODO: fix time stamp below!
	bufferItem.timestamp =  Timestamp(currentTime()); // Timestamp(ros::Time::now().toSec());
  // fprintf(stderr,"currentTime: %.3f\n",currentTime());

	if(undistorter != 0)
	{
		assert(undistorter->isValid());
		// undistorter->undistort(cv_ptr->image,bufferItem.data);
		undistorter->undistort(img,bufferItem.data);
	}
	else
	{
		bufferItem.data = img; //cv_ptr->image;
	}

	imageBuffer->pushBack(bufferItem);
}

void OCVImageStreamThread::infoCb(cv::VideoCapture info)
{
	if(!haveCalib)
	{
	// 	fx_ = info->P[0];
	// 	fy_ = info->P[5];
	// 	cx_ = info->P[2];
	// 	cy_ = info->P[6];

	// 	if(fx_ == 0 || fy_==0)
	// 	{
	// 		printf("camera calib from P seems wrong, trying calib from K\n");
	// 		fx_ = info->K[0];
	// 		fy_ = info->K[4];
	// 		cx_ = info->K[2];
	// 		cy_ = info->K[5];
	// 	}

	// 	width_ = info->width;
	// 	height_ = info->height;

    fx_ = 1;
    fy_ = 1;
    width_ = info.get(CV_CAP_PROP_FRAME_WIDTH);
    height_ = info.get(CV_CAP_PROP_FRAME_HEIGHT);
    cx_ = width_*.5;
    cy_ = height_*.5;

		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
	}
}

}
