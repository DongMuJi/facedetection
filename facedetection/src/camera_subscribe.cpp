/* ***********************************************
Author        :guanjunace@foxmail.com
Created Time  :2017年07月10日 星期一 10时43分26秒
File Name     :camera_subscribe.cpp
************************************************ */
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;
using namespace cv;
const string cascadeName = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
const string nestedCascadeName = "/usr/share/opencv/haarcascades/haarcascade_eye.xml";
const double scale = 1.3;
void detectFace(Mat& img, CascadeClassifier& cascade,
		CascadeClassifier& nestedCascade) {
	double t = 0;
	vector<Rect> faces;
	const static Scalar colors[] = {
		Scalar(255,0,0),
		Scalar(255,128,0),
		Scalar(255,255,0),
		Scalar(0,255,0),
		Scalar(0,128,255),
		Scalar(0,255,255),
		Scalar(0,0,255),
		Scalar(255,0,255)
	};
	Mat gray, smallImg;

	cvtColor( img, gray, COLOR_BGR2GRAY );
	double fx = 1 / scale;
	resize(gray, smallImg, Size(), fx, fx, INTER_LINEAR);
	equalizeHist(smallImg, smallImg);
	t = (double)getTickCount();
    cascade.detectMultiScale(smallImg, faces, 1.1, 2, 0
			//|CASCADE_FIND_BIGGEST_OBJECT
			//|CASCADE_DO_ROUGH_SEARCH
			|CASCADE_SCALE_IMAGE,
			Size(30, 30));
	t = (double)getTickCount() - t;
	printf("detection time = %g ms\n", t*1000/getTickFrequency());	
	for (size_t i = 0; i < faces.size(); ++i) {
		Rect r = faces[i];
		Mat smallImgROI;
		vector<Rect> nestedObjects;
		Point center;
		Scalar color = colors[i%8];
		int radius;
		double aspect_ratio = (double)r.width/r.height;
		if(0.75 < aspect_ratio && aspect_ratio < 1.3) {
			center.x = cvRound((r.x + r.width*0.5)*scale);
			center.y = cvRound((r.y + r.height*0.5)*scale);
			radius = cvRound((r.width + r.height)*0.25*scale);
			circle(img, center, radius, color, 3, 8, 0);
		} else {
			rectangle(img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
					cvPoint(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
					color, 3, 8, 0);
		}
		smallImgROI = smallImg(r);
		nestedCascade.detectMultiScale(smallImgROI, nestedObjects,
				1.1, 2, 0
				|CASCADE_SCALE_IMAGE,
				Size(30, 30) );
		for (size_t j = 0; j < nestedObjects.size(); ++j) {
			Rect nr = nestedObjects[j];
			center.x = cvRound((r.x + nr.x + nr.width*0.5)*scale);
			center.y = cvRound((r.y + nr.y + nr.height*0.5)*scale);
			radius = cvRound((nr.width + nr.height)*0.25*scale);
			circle(img, center, radius, color, 3, 8, 0);
		}

	}
	imshow("result", img);
	waitKey(1);

}
void img_Callback(const sensor_msgs::ImageConstPtr& msg) {
	Mat image ;
	CascadeClassifier cascade, nestedCascade;
	try {
		image = cv_bridge::toCvShare(msg, "bgr8")->image;
		//CascadeClassifier cascade, nestedCascade;
		nestedCascade.load(nestedCascadeName);
		cascade.load(cascadeName);
		detectFace(image, cascade, nestedCascade);
		//Convert an immutable sensor_msgs::Image message to an OpenCV-compatible CvImage, 
		//sharing the image data if possible. 
		//imshow("img", cv_bridge::toCvShare(msg, "bgr8")->image);//mat
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	if (!image.empty()) detectFace(image, cascade, nestedCascade);
	else printf("no image!");
}
int main(int argc, char *argv[]) { 
	ros::init(argc, argv, "detectFace");
	ros::NodeHandle nh;
	namedWindow("camera");
	startWindowThread();/*
	CascadeClassifier cascade, nestedCascade;
	if (!nestedCascade.load(nestedCascadeName))
		cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
	if (!cascade.load(cascadeName)) {
		cerr << "ERROR: Could not load classifier cascade" << endl;
		return -1;
   	}
*/
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub = it.subscribe("/camera/rgb/image_raw", 1, &(img_Callback));

	destroyWindow("camera");
	ros::spin();
	return 0;
}
