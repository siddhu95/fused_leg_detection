#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/gui_widgets.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>  

using namespace std;
using namespace dlib;
std::string rgb_im_topic = "/camera/rgb/image_rect_color";
std::string depth_im_topic = "/camera/depth/image";
std::string svm_path = "/home/miningrox/Workspaces/test_ws/src/dlib/src/examples/build/face_detector.svm";
float min1=0.5,max1=5.5,min2=7.0,max2=13.0;
float scale_x=2,scale_y=3,adj_thresh=-0.4;


inline float StringToNumber ( const string &Text )
{
    istringstream ss(Text);
    float result;
    return ss >> result ? result : 0;
}
// ----------------------------------------------------------------------------------------
class listento
{
    typedef scan_fhog_pyramid<pyramid_down<10> > image_scanner_type; 
    typedef matrix<double,0,1> feature_vector_type;
    typedef std::vector<std::pair<double, rectangle> > rex;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;

    ros::NodeHandle n;
    //ros::Subscriber s;
    object_detector<image_scanner_type> detector,detector2;
    image_window win1;
    image_window win2;
    image_window winhog1;
    image_window winhog2;
    image_window hogwin; 
    array2d<matrix<float,31,1> > hog;

    message_filters::Subscriber<sensor_msgs::Image> rgb_im;
    message_filters::Subscriber<sensor_msgs::Image> depth_im;
    message_filters::Synchronizer<MySyncPolicy> sync;
      

        
public:
    listento():\
    n(),\
    rgb_im(n,rgb_im_topic,10),\
    depth_im(n,depth_im_topic,10),\
    sync(MySyncPolicy(10),rgb_im, depth_im)\
    {

        cout<<"hello";
        sync.registerCallback(boost::bind(&listento::rgbd_hog_callback,this,_1,_2));
        //s = n.subscribe(depth_im_topic,10,&listento::depth_view,this);
        deserialize(svm_path) >> detector2;
        detector = detector2;//threshold_filter_singular_values(detector2,0.05);   
        hogwin.set_image(draw_fhog(detector));

    }
    void rgbd_hog_callback(const sensor_msgs::ImageConstPtr& img_rgb,const sensor_msgs::ImageConstPtr& img_depth)
    {
        cv_bridge::CvImagePtr bridge_rgb,bridge_depth;
        
        try{
            bridge_rgb = cv_bridge::toCvCopy(img_rgb,"rgb8");//"32FC1");
            bridge_depth = cv_bridge::toCvCopy(img_depth,"32FC1");
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR_STREAM("Failed to transform image");
            return;
        }
        process_rgbd_hog(bridge_rgb->image, bridge_depth->image, min1, max1, &win1, &winhog1);
        process_rgbd_hog(bridge_rgb->image, bridge_depth->image, min2, max2, &win2, &winhog2);
    }
    void process_rgbd_hog(cv::Mat img_rgb,cv::Mat img_depth,float min,float max,image_window* win, image_window* winhog)
    {
        //scale_y=scale_x=1;
        typedef float type_of_depth;
        float conv_f=1;
        array2d<rgb_alpha_pixel> rgbd_img(long(img_rgb.rows),long(img_rgb.cols));
        array2d<rgb_pixel> rgb_img(long(img_rgb.rows),long(img_rgb.cols));
        array2d<rgb_alpha_pixel> rgbd_img_scaled(long(img_rgb.rows/scale_y),long(img_rgb.cols/scale_x));
        array2d<rgb_pixel> rgb_img_scaled(long(img_rgb.rows/scale_y),long(img_rgb.cols/scale_x));
        array2d<char> depth_im(long(img_rgb.rows),long(img_rgb.cols));
        array2d<char> depth_im_scaled(long(img_rgb.rows/scale_y),long(img_rgb.cols/scale_x));
        type_of_depth *ptr_depth;
        unsigned char *ptr_rgb;
        for(long i = 0; i < rgbd_img.nr(); i++)
        {
            ptr_rgb = img_rgb.ptr<unsigned char>(i);
            ptr_depth = img_depth.ptr<type_of_depth>(i);
            for(long j = 0; j < rgbd_img.nc(); j++)
            {   
                rgbd_img[i][j].alpha = (char) (255*((ptr_depth[j]/conv_f-min)/(max-min)));
                rgbd_img[i][j].red = ptr_rgb[j*3];
                rgbd_img[i][j].green = ptr_rgb[j*3+1];
                rgbd_img[i][j].blue = ptr_rgb[j*3+2];
                rgb_img[i][j].red = ptr_rgb[j*3];
                rgb_img[i][j].green = ptr_rgb[j*3+1];
                rgb_img[i][j].blue = ptr_rgb[j*3+2];

            }  
        }
        for(int i = 0; i < img_depth.rows; i++)
        {
            type_of_depth* Di = img_depth.ptr<type_of_depth>(i);
            for(int j = 0; j < img_depth.cols; j++)
            {   
                depth_im[i][j] = (char) (255*((Di[j]/conv_f-5.5)/(14-5.5)));
            }
        }
        resize_image(rgbd_img,rgbd_img_scaled);
        resize_image(rgb_img,rgb_img_scaled);
        resize_image(depth_im,depth_im_scaled);
        rex fulldets;
        std::vector<rectangle> dets;

        detector(rgbd_img_scaled,fulldets,adj_thresh);
        for(rex::iterator i=fulldets.begin();i!=fulldets.end();i++)
        {
            dets.push_back(i->second);
        }

        if(fulldets.size())
            for(int j=0;j<fulldets.size();j++)
            {
                cout<<"detection "<<j<<" confidence : "<<fulldets[j].first<<endl;
            }
        win->clear_overlay();
        win->set_image(depth_im_scaled);//rgb_img_scaled);
        win->add_overlay(dets, rgb_pixel(255,0,0));
        extract_fhog_features(rgbd_img_scaled, hog);
        //cout << "hog image has " << hog.nr() << " rows and " << hog.nc() << " columns." << endl;
        // Let's see what the image and FHOG features look like.
        winhog->set_image(draw_fhog(hog));
    }
};






int main(int argc, char** argv)
{  
    try
    {

        ros::init(argc, argv, "RGBD_hog");
        ros::param::get("~min1",min1);
        ros::param::get("~max1",max1);
        ros::param::get("~min2",min2);
        ros::param::get("~max2",max2);
        ros::param::get("~scale_x",scale_x);
        ros::param::get("~scale_y",scale_y);
        ros::param::get("~adj_thresh",adj_thresh);
        ros::param::get("~rgb_im_topic",rgb_im_topic);
        ros::param::get("~depth_im_topic",depth_im_topic);
        ros::param::get("~svm_path",svm_path);
        listento L;
        ros::spin();        
    }
    catch (exception& e)
    {
        cout << "\nexception thrown!" << endl;
        cout << e.what() << endl;
    }
}

// ----------------------------------------------------------------------------------------

