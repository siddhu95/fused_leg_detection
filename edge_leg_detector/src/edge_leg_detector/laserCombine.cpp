    #include <ros/ros.h>
    #include <sensor_msgs/LaserScan.h>
    #include <geometry_msgs/PoseArray.h>
    #include <iostream>
    #include <cmath>
    #include <vector>
    #include <list>
    #include <string>
    #include <message_filters/subscriber.h>
    #include <message_filters/time_synchronizer.h>

    #include <message_filters/synchronizer.h>
    #include <message_filters/sync_policies/approximate_time.h>


    #include <tf/transform_listener.h> 
    #include <boost/bind.hpp>  
    #include <boost/filesystem.hpp>
   
    using namespace std;

    int hp= 0;  
    double Dist2D( geometry_msgs::Point, geometry_msgs::Point);

    class laser_comb
    {
    	tf::TransformListener listener;
    	ros::Publisher pos_pub;
    	public:
    	laser_comb(ros::NodeHandle *n,std::string publish_topic):pos_pub(),listener(ros::Duration(10))
    	{
    		pos_pub = n->advertise<geometry_msgs::PoseArray>(publish_topic,int(10));
    	}
    	
    	void transform_coord(std::string,const geometry_msgs::PoseArray::ConstPtr&,geometry_msgs::PoseArray*);
    	void callback(const geometry_msgs::PoseArray::ConstPtr& ,const geometry_msgs::PoseArray::ConstPtr& );
    };

    void laser_comb::callback(const geometry_msgs::PoseArray::ConstPtr& p_array1,const geometry_msgs::PoseArray::ConstPtr& p_array2)
    {
        //ROS_INFO_STREAM("hi");
    	geometry_msgs::PoseArray p_array3;
    	if (!(p_array1->poses.empty()||p_array2->poses.empty())&&(hp<20))
    	{	
            if(p_array2->poses.size()==p_array1->poses.size())
    		{
                 hp++;
    			transform_coord(p_array2->header.frame_id,p_array1,&p_array3);
    			int p = p_array2->poses.size();
                ROS_INFO_STREAM("performing size:"<<p<<"hp::"<<hp);
    			//list<geometry_msgs::Pose>::iterator it = p_array3.poses.begin();
                for (int i = 0;i<p;i++)
                    ROS_INFO_STREAM('\n'<<p_array2->poses[i].position<<'\n'<<p_array3.poses[i].position);
    			for (int i = 0;i<p;i++)
    			{
    				if(Dist2D(p_array2->poses[i].position,p_array3.poses[i].position)>0.3f)
                    ROS_INFO_STREAM("\n***Deleted :"<<Dist2D(p_array2->poses[i].position,p_array3.poses[i].position));
    				p_array3.poses.erase(p_array3.poses.begin()+i);

    			}
            
    			pos_pub.publish(p_array3);
                
    		}
            else
                ;//ROS_INFO_STREAM("sizes not equal : "<<p_array1->poses.size()<<"and"<<p_array2->poses.size());
        }
        else
            ;//ROS_INFO_STREAM("empty");

        
    }
    void laser_comb::transform_coord(std::string frameid,const geometry_msgs::PoseArray::ConstPtr& p_array1,geometry_msgs::PoseArray *p_array2)
    {

        
    	geometry_msgs::PointStamped old_point,new_point;
    	geometry_msgs::Quaternion HumanQuaternion;
    	HumanQuaternion.x = 0;//|-> Orientation is ignored
    	HumanQuaternion.y = 0;//|
    	HumanQuaternion.z = 0;//|
    	HumanQuaternion.w = 1;//|
    	geometry_msgs::Pose HumanPose;
    	HumanPose.orientation= HumanQuaternion;

        old_point.header = p_array1->header;
    	int p = p_array1->poses.size();

    	for (int i = p-1;i>=0;i--)
    	{
    	 	old_point.point = p_array1->poses[i].position;
    	 	/*old_point.point.y = p_array1->poses[i].position.y;
    	 	old_point.point.z = p_array1->poses[i].position.z;*/
    	 	//ROS_INFO_STREAM("hello");
            try{
                
                listener.waitForTransform(frameid,old_point.header.frame_id,old_point.header.stamp,ros::Duration(0.3));
    	 		listener.transformPoint(frameid,old_point,new_point);
                HumanPose.position = new_point.point;
                p_array2->poses.push_back(HumanPose);
                p_array2->header = new_point.header;


    	 	}
    		catch(tf::TransformException& ex){
      			ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    		}
      	
      		

    	}

    }
    double Dist2D( geometry_msgs::Point A, geometry_msgs::Point B){
      return sqrt( pow( A.x - B.x, 2 ) + pow( A.y - B.y, 2 ) );
    }
    int main(int argc, char** argv)
    {
    	ros::init(argc, argv, "laser_combine");
    	ros::NodeHandle n;
        std::string Posearray1_topic = "/posearray1";
        std::string Posearray2_topic = "/posearray2";
        std::string Combined_Posearray_topic ="/combined_posearray";
        ros::param::get("~Posearray1_topic",Posearray1_topic);
        ros::param::get("~Posearray2_topic",Posearray2_topic);
        ros::param::get("~Combined_Posearray_topic",Combined_Posearray_topic);

    	laser_comb p(&n,Combined_Posearray_topic);
    	
    	message_filters::Subscriber<geometry_msgs::PoseArray> laser1_sub(n,Posearray1_topic,10);//Set param laser1 and laser2 to the topic names of 
    	message_filters::Subscriber<geometry_msgs::PoseArray> laser2_sub(n,Posearray2_topic,10);//the two laser scans you want to merge!!!
    	//message_filters::TimeSynchronizer<geometry_msgs::PoseArray,geometry_msgs::PoseArray> sync(laser1_sub, laser2_sub, 100);
    	//boost::function<void (const geometry_msgs::PoseArray::ConstPtr&,const geometry_msgs::PoseArray::ConstPtr&)> funcobj = boost::bind(&laser_comb::callback,p,_1,_2);
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray,geometry_msgs::PoseArray> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),laser1_sub, laser2_sub);

    	sync.registerCallback(boost::bind(&laser_comb::callback,&p,_1,_2));

    	ros::spin();

    	return 0;

    }