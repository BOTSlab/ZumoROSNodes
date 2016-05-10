/*
 * pixy_node is part of the pixy_ros package for interfacing with
 * a CMUcam5 pixy with ROS.
 * Copyright (C) 2014 Justin Eskesen
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ros/ros.h>
// File doesn't exist. However, this was only used to convert from degress to radians so NBD.
//#include <angles/angles.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <pixy_node/PixyData.h>
#include <pixy_node/PixyBlock.h>
#include <pixy_node/Servo.h>

#include "pixy.h"
#define BLOCK_BUFFER_SIZE 100

class PixyNode
{
public:
	PixyNode();

	void spin();

private:
	void update();
	void setServo(const pixy_node::Servo& msg) {pixy_rcs_set_position(msg.channel, msg.position);}

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;

	ros::Rate rate_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher publisher_;
	ros::Subscriber servo_subscriber_;
	std::string frame_id;

	bool use_servos_;

};

PixyNode::PixyNode() :
		node_handle_(),
		private_node_handle_("~"),
		use_servos_(false),
		rate_(10.0)
{

	private_node_handle_.param<std::string>(std::string("frame_id"), frame_id,
			std::string("pixy_frame"));

	double rate;
	private_node_handle_.param("rate", rate, 50.0);
	rate_=ros::Rate(rate);

    private_node_handle_.param("use_servos", use_servos_, false);

    if(use_servos_)
    {
        servo_subscriber_ = node_handle_.subscribe("servo_cmd", 20, &PixyNode::setServo, this);
    }

    int ret = pixy_init();
    if (ret != 0)
      {
	ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
		  __FUNCTION__, ret);
	ROS_BREAK();
      }
    publisher_ = node_handle_.advertise<pixy_node::PixyData>("block_data", 50.0);
    

}



void PixyNode::update()
{

	// Pixy Block buffer //
	struct Block blocks[BLOCK_BUFFER_SIZE];

	// Get blocks from Pixy //
	int blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, blocks);

	pixy_node::PixyData data;

	if (blocks_copied > 0)
	{
		data.header.stamp = ros::Time::now();
		for (int i = 0; i < blocks_copied; i++)
		{
			pixy_node::PixyBlock pixy_block;
			pixy_block.type = blocks[i].type;
			pixy_block.signature = blocks[i].signature;
			pixy_block.roi.x_offset = blocks[i].x;
			pixy_block.roi.y_offset = blocks[i].y;
			pixy_block.roi.height = blocks[i].height;
			pixy_block.roi.width = blocks[i].width;
			pixy_block.roi.do_rectify = false;
			
			// The header angles.h does not exist. I am commenting this out and will 
			// code the conversion from degrees to radians here.
			//
			// - Nicholi.
			//
			//pixy_block.angle =
			//		(pixy_block.type == TYPE_COLOR_CODE) ?
			//				angles::from_degrees((double) blocks[i].angle) :
			//				0.0;

			pixy_block.angle = (pixy_block.type == TYPE_COLOR_CODE) ? blocks[i].angle * M_PI / 180. : 0.0;

			data.blocks.push_back(pixy_block);
		}

	}
	else if(blocks_copied < 0)
	{
		ROS_INFO("Pixy read error.");
		return;
	}

	// publish the message
	publisher_.publish(data);
}


void PixyNode::spin()
{

	while (node_handle_.ok())
	{
		update();

		ros::spinOnce();
		rate_.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pixy_node");

	ROS_INFO("PixyNode for ROS");

	PixyNode myPixy;
	myPixy.spin();

	return (0);
}

// EOF
