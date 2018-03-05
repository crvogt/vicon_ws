#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class SmoothingNode{
private:
	int counter;
	int slope;
	float num, denom;
	ros::NodeHandle *nh_;

	geometry_msgs::PoseStamped inData1;
	geometry_msgs::PoseStamped inData2;
	geometry_msgs::TransformStamped outData;

	ros::Subscriber getPoseSub;
	ros::Publisher repub_pub;

public:
	SmoothingNode(ros::NodeHandle *nh);
	void rosSetup();

	void getPoseCallback(const geometry_msgs::PoseStamped &msg);
};

SmoothingNode::SmoothingNode(ros::NodeHandle *nh) : nh_(nh){
	this->rosSetup();
}

void SmoothingNode::rosSetup(){
	getPoseSub = nh_->subscribe("vrpn_client_node/carson_quad_2/pose", 1000, 
		&SmoothingNode::getPoseCallback, this);
	repub_pub = nh_->advertise<geometry_msgs::TransformStamped>("/vicon/carson_quad_2/carson_quad_2", 1000);

	//let counter build
	counter = 0;
}

void SmoothingNode::getPoseCallback(const geometry_msgs::PoseStamped &msg){
	
	if(counter < 200){
		inData1 = msg;
		inData2 = msg;
	}
	else{
		inData1 = inData2;
		inData2 = msg;
		//get a slope calculation;
		num = inData2.pose.position.y - inData1.pose.position.y;
		denom = (float)(inData2.header.stamp.toSec() - inData1.header.stamp.toSec());
		slope = std::abs(num / denom);
		if(slope > 15){
			inData2 = inData1;
		}
		outData.header.stamp = ros::Time::now();
		outData.header.frame_id = "/world";
		outData.child_frame_id = "/vicon/carson_quad_2/carson_quad_2";
		outData.transform.translation.x = inData2.pose.position.x;
		outData.transform.translation.y = inData2.pose.position.y;
		outData.transform.translation.z = inData2.pose.position.z;
		outData.transform.rotation.x = inData2.pose.orientation.x;
		outData.transform.rotation.y = inData2.pose.orientation.y;
		outData.transform.rotation.z = inData2.pose.orientation.z;
		outData.transform.rotation.w = inData2.pose.orientation.w;

		repub_pub.publish(outData);
	}

}

int main(int argc, char **argv){
	ros::init(argc, argv, "smoothing_node");
	ros::NodeHandle nh;

	ros::Rate rate(100);

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}

	return 1;
}