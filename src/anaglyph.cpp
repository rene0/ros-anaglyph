/*
Copyright (c) 2010 René Ladan. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv/highgui.h>

int g_offset_h, g_offset_v, g_switch_h, g_switch_v;
image_transport::Publisher g_pub;
bool got_pic = false;
unsigned int g_width = 0, g_height = 0;

void anaglyph(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
	sensor_msgs::Image ana;
	unsigned int j, r;
	int l;

	if (!got_pic)
	{
		g_width = left->width;
		g_height = left->height;
	}

	ana.header = left->header;
	ana.width = left->width - g_offset_h;
	ana.height = left->height - g_offset_v;
	ana.encoding = "bgr8";
	ana.is_bigendian = 0;
	ana.step = ana.width * 3;
	ana.data.resize(ana.height*ana.step);
	for (unsigned int y = 0; y < ana.height; y++)
		for (unsigned int x = 0; x < ana.step; x += 3)
		{
			j = ana.step*y + x;
			r = right->step*y + x;
			l = r + 3 * g_offset_h * (g_switch_h ? 1 : -1)
			      + left->step * g_offset_v * (g_switch_v ? 1 : -1);
			if ((unsigned)l > left->step * left->height)
			{
				l = left->step * left->height - 3;
				ROS_WARN("left clipped +");
			}
			if (l < 0)
			{
				l = 0;
				ROS_WARN("left clipped -");
			}
			// copy anaglyph (green) channel
			ana.data[j+1] = left->data[l+1];
			// copy normal channels
			ana.data[j+0] = right->data[r+0];
			ana.data[j+2] = right->data[r+2];
		}
	g_pub.publish(ana);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "anaglyph");
	ros::NodeHandle nh("~");
	nh.param("horizontal", g_offset_h, 0);
	nh.param("vertical", g_offset_v, 0);
	nh.param("pos_h", g_switch_h, 1);
	nh.param("pos_v", g_switch_v, 1);

	image_transport::ImageTransport it(nh);
	message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "left", 1);
	message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "right", 1);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_sub, right_sub, 10); // 10 ms?
	sync.registerCallback(boost::bind(&anaglyph, _1, _2));

	cvNamedWindow("Anaglyph Control", 0);
	cvCreateTrackbar("Horizontal positive", "Anaglyph Control", &g_switch_h, 1, NULL);
	cvCreateTrackbar("Vertical positive", "Anaglyph Control", &g_switch_v, 1, NULL);

	g_pub = it.advertise("image_anaglyph", 1);

	while (nh.ok())
	{
		ros::spinOnce(); // process the images
		if (g_width != 0)
		{
			// For some reason, image_view thinks that a 1x1 image
			// is wrong, so limit the minimal image to 2x2
			cvCreateTrackbar("Horizontal", "Anaglyph Control", &g_offset_h, g_width-2, NULL);
			cvCreateTrackbar("Vertical", "Anaglyph Control", &g_offset_v, g_height-2, NULL);
			got_pic = true;
		}
		cvWaitKey(10);
		// use this instead of cvStartWindowThread()
		// because of the better response time
	}

	cvDestroyWindow("Anaglyph Control");
	return 0;
}
