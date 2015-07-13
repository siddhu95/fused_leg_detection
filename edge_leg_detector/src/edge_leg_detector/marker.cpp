#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include <bitset>

class marker_class
{

	ros::Publisher vis_pub;
	public:
	marker_class(ros::NodeHandle* n):vis_pub()
	{
		vis_pub = n->advertise<visualization_msgs::Marker>( "visualization_marker", 10);


	}
	void visCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
};


void marker_class::visCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	// namespace pos = msg->poses[0].position;
	// namespace ori = msg->poses[0].orientation;
	if(!msg->poses.empty())
	{
		visualization_msgs::Marker marker;
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

			vis_pub.publish( marker );
			//ROS_INFO_STREAM("Published "<<i<<" "<<msg->poses.size()<<'\n');
			
	    }
	}
}

	

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "marker_node");
	ros::NodeHandle n;
    marker_class M(&n);
    ros::Subscriber sub = n.subscribe("/DetectedPersons1_position", 0, &marker_class::visCallback, &M);
	ros::spin();
    return 0;
}

    
