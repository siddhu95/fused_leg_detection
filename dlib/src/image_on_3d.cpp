#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/opencv.h>


#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <limits.h>
#include <sstream>
#include <fstream> 
#include <stdlib.h>
#include <boost/bind.hpp>  
#include <visualization_msgs/Marker.h>




#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace std;
using namespace dlib;
std::string ip_topic = "/camera/rgb/image_rect_color";
std::string ip_topic_depth = "/camera/depth/image_raw";
std::string op_topic ="/Image_on_3d";
std::string cam_info ="/camera/camera_info";
std::string svm_path = "/home/miningrox/Workspaces/test_ws/src/dlib/src/examples/build/face_detector.svm";

float scale_x=2,scale_y=3,adj_thresh=-0.4;


class Image_on_3d
{
	typedef scan_fhog_pyramid<pyramid_down<10> > image_scanner_type; 
    typedef matrix<double,0,1> feature_vector_type;
    typedef std::vector<std::pair<double, rectangle> > rex;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;

    ros::NodeHandle n;
    //tf::TransformListener listener;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher marker_pub;
    object_detector<image_scanner_type> detector,detector2;
    image_window hogwin,win; 
    message_filters::Subscriber<sensor_msgs::Image> rgb_im;
    message_filters::Subscriber<sensor_msgs::Image> depth_im;
    message_filters::Synchronizer<MySyncPolicy> sync;
    image_geometry::PinholeCameraModel cam_model;
    int l;
    visualization_msgs::Marker points, line_strip, line_list;
    /*cv::Mat intrisicMat;// Intrinsic parameter matrix
    cv::Mat rVec; // Rotation vector
    cv::Mat tVec; // Translation vector
    cv::Mat distCoeffs;   // Distortion vector
    cv::Mat rVecR;//Rodriguez rotation matrix*/


public:
    Image_on_3d():\
    rgb_im(n,ip_topic,10),\
    depth_im(n,ip_topic_depth,10),\
    sync(MySyncPolicy(10),rgb_im, depth_im),\
    l(0)
    {
    	sub = n.subscribe(cam_info, 10, &Image_on_3d::cam_set, this);
        pub = n.advertise<geometry_msgs::PoseArray>(op_topic,int(10));
        deserialize(svm_path) >> detector2;
        detector = detector2;//threshold_filter_singular_values(detector2,0.05);   
        hogwin.set_image(draw_fhog(detector));
        sync.registerCallback(boost::bind(&Image_on_3d::callback,this,_1,_2));
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        



        
        /*visualization_msgs::Marker marker;
		marker.header.frame_id = msg->header.frame_id;//"base_link";
		marker.header.stamp = msg->header.stamp;//ros::Time();
		marker.ns = "rviz_marker";
		marker.id = 252;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		for(int i = 0;i<msg->poses.size();i++)
		{	
			std::bitset<3> num(i);
			marker.pose.position.x = msg->poses[i].position.x;
			marker.pose.position.y = msg->poses[i].position.y;
			marker.pose.position.z = msg->poses[i].position.z;
			marker.pose.orientation.x = msg->poses[i].orientation.x;
			marker.pose.orientation.y = msg->poses[i].orientation.y;
			marker.pose.orientation.z = msg->poses[i].orientation.z;
			marker.pose.orientation.w = msg->poses[i].orientation.w;
			marker.scale.x = 0.11;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = float(num[0]);
			marker.color.g = float(num[1]);
			marker.color.b = float(num[2]);

			vis_pub.publish( marker );*/

        /*intrisicMat.at<double>(0, 0) = 544.943281;//577.7485574387038;
        intrisicMat.at<double>(1, 0) = 0;
        intrisicMat.at<double>(2, 0) = 0;
        intrisicMat.at<double>(0, 1) = 0;
        intrisicMat.at<double>(1, 1) = 554.248901;//586.6264017847345;
        intrisicMat.at<double>(2, 1) = 0;
        intrisicMat.at<double>(0, 2) = 332.619586;//329.4045569544355;
        intrisicMat.at<double>(1, 2) = 225.534167;//230.398241073748;
        intrisicMat.at<double>(2, 2) = 1;

        rVec.at<double>(0) = 0;
        rVec.at<double>(1) = 0;
        rVec.at<double>(2) = 0;
        cv::Rodrigues(rVec,rVecR);
        cout<<"######################################################################################################################hiGuys"<<rVecR.at<double>(0)<<" "<<rVecR.at<double>(1)<<" "<<rVecR.at<double>(2)<<endl;


        tVec.at<double>(0) = 0;
        tVec.at<double>(1) = 0;
        tVec.at<double>(2) = 0;

        distCoeffs.at<double>(0) = 0.033229;//-0.01365372077899292;
        distCoeffs.at<double>(1) = -0.06397;//0.03892826855838201;
        distCoeffs.at<double>(2) = -0.008713;//-0.009254581909543487;
        distCoeffs.at<double>(3) = 0.009804;//0.007533077555784945;
        distCoeffs.at<double>(4) = 0;*/
    }
    geometry_msgs::Point calc3dpt(cv::Point3d pt,float dist)
    {
    
    	float alpha = dist/(pt.z+0.000001);
        //alpha=1;
        cout<<"dist="<<dist<<"    alpha="<<alpha<<endl;
    	geometry_msgs::Point pt_3d;
    	pt_3d.x = alpha*pt.x;
    	pt_3d.y = alpha*pt.y;
    	pt_3d.z = alpha*pt.z;
        ROS_INFO_STREAM("My::"<<pt_3d);
    	return pt_3d;
    }



void callback(const sensor_msgs::Image::ConstPtr& img,const sensor_msgs::Image::ConstPtr& imgd)
{   
    if(!l)
        return;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = img->header.frame_id;
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    cv_bridge::CvImagePtr bridge,bridged;
    try{
        bridge = cv_bridge::toCvCopy(img,"bgr8");
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR_STREAM("Failed to trransform d_image in depth_record()");
        return;
    }
    try{
        bridged = cv_bridge::toCvCopy(imgd,"16UC1");
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR_STREAM("Failed to trransform d_image in depth_record()");
        return;
    }
    cv::Mat tmp;
    geometry_msgs::Pose Pose_;
    geometry_msgs::Point Q;
    cv::Point3d Pt;
    geometry_msgs::PoseArray P;
	Pose_.orientation.x = 0;//|-> Orientation is ignored
	Pose_.orientation.y = 0;//|
	Pose_.orientation.z = 0;//|
	Pose_.orientation.w = 1;//|
    P.header=img->header;
    float x,y,z;
  
    cv::Mat conv_img(bridge->image.rows, bridge->image.cols, CV_8UC1);
    cv::Mat conv_img_abs(int(bridge->image.rows/scale_y), int(bridge->image.cols/scale_x), CV_16UC1);
        for(int i = 0; i < bridged->image.rows; i++)
        {
            unsigned char* Di = bridged->image.ptr<unsigned char>(i);
            char* Ii = conv_img.ptr<char>(i);
            for(int j = 0; j < bridged->image.cols; j++)
            {   
                Ii[j] = (char) (255*((Di[j]-5.5)/(14-5.5)));
            }
            //cout<<Ii[0]<<" ";
        }

    cv::resize(conv_img,tmp,cv::Size(int(bridge->image.rows/scale_y),int(bridge->image.cols/scale_x)));
    cv::resize(bridged->image,conv_img_abs,cv::Size(int(bridge->image.rows/scale_y),int(bridge->image.cols/scale_x)));
    cv_image<unsigned char> cimg(tmp);
    //resize_image(img,cimg);
    
    rex fulldets;
    std::vector<rectangle> dets;

    detector(cimg,fulldets,adj_thresh);
    for(rex::iterator i=fulldets.begin();i!=fulldets.end();i++)
    {
        dets.push_back(i->second);
    }

    if(fulldets.size())
    {
        for(int j=0;j<fulldets.size();j++)
        {
        	x = (int)(fulldets[j].second.left()+fulldets[j].second.right())/2;
        	y = (int)(fulldets[j].second.top()+fulldets[j].second.bottom())/2;
            cv::Point2d pixel(x,y);
  			Pt =  cam_model.projectPixelTo3dRay(pixel);
                                                            for (uint32_t i = 0; i < 20; ++i)
                                                            {
                                                                geometry_msgs::Point p;
                                                                p.x = i*Pt.x;
                                                                p.y = i*Pt.y;
                                                                p.z = i*Pt.z;

                                                                points.points.push_back(p);
                                                                line_strip.points.push_back(p);
                                                            }


                                                            marker_pub.publish(points);
                                                            points.points.clear();
                                                            marker_pub.publish(line_strip);
                                                            line_strip.points.clear();
                                                            //marker_pub.publish(line_list);

        	Pose_.position = calc3dpt(Pt,float(conv_img_abs.at<float>(x,y)));
        	P.poses.push_back(Pose_);
            cout<<"detection "<<j<<" confidence : "<<fulldets[j].first<<endl;
        }
        pub.publish(P);
    }
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(dets, rgb_pixel(255,0,0));
    //extract_fhog_features(cimg, hog);
    //cout << "hog image has " << hog.nr() << " rows and " << hog.nc() << " columns." << endl;
    // Let's see what the image and FHOG features look like.
    //winhog->set_image(draw_fhog(hog));

}
void cam_set(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if(!l)
    {
        ROS_INFO_STREAM("Camera MESSAHE");
        cam_model.fromCameraInfo(info_msg);
        l=1;
    }
}


};


int main(int argc, char** argv)
{

    ros::init(argc, argv, "Image_on_3d");

    ros::param::get("~ip_topic",ip_topic);
    ros::param::get("~ip_topic_depth",ip_topic_depth);
    ros::param::get("~op_topic",op_topic);
    ros::param::get("~scale_x",scale_x);
    ros::param::get("~scale_y",scale_y);
    ros::param::get("~adj_thresh",adj_thresh);
    ros::param::get("~cam_info",cam_info);
    ros::param::get("~svm_path",svm_path);

    Image_on_3d p;
    ros::spin();
    return 0;
}



    
