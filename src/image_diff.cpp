/*!
  \file       image_diff.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/11/19

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

\section Subscriptions
  - \b "~in1", "~in2"
    [sensor_msgs/Image]

\section Publications
  - \b "~out"
    [sensor_msgs/Image]
*/
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// use of message filters - inspiration available at
// http://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
typedef image_transport::SubscriberFilter ImgSub;
typedef sensor_msgs::Image Image;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

image_transport::Publisher _out_pub;
cv_bridge::CvImageConstPtr _b1, _b2;
cv_bridge::CvImage _bout;
cv::Mat1b _i1bw, _i2bw, _iout;
double _scale = 1;

void callback(const Image::ConstPtr& i1,
              const Image::ConstPtr& i2) {
  if (i1->width != i2->width  || i1->height != i2->height) {
    ROS_WARN("Images of different sizes: (%i, %i), (%i, %i)",
             i1->width, i2->width, i1->height, i2->height);
    return;
  }
  if (!i1->width || !i1->height) {
    ROS_WARN("Empty image (%i, %i)", i1->width, i1->height);
    return;
  }
  _b1 = cv_bridge::toCvShare(i1);
  _b2 = cv_bridge::toCvShare(i2);
  if (_b1->image.channels() == 1)
    _b1->image.copyTo(_i1bw);
  else
    cv::cvtColor(_b1->image, _i1bw, CV_BGR2GRAY);
  if (_b2->image.channels() == 1)
    _b2->image.copyTo(_i2bw);
  else
    cv::cvtColor(_b2->image, _i2bw, CV_BGR2GRAY);
  cv::absdiff(_i1bw, _i2bw, _iout);
  if (fabs(_scale - 1) > 1E-2)
    _iout *= _scale;
  _iout.copyTo(_bout.image);
  _bout.encoding = "mono8";
  _bout.header = i1->header;
  _out_pub.publish(_bout.toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_diff");
  ros::NodeHandle nh_private("~");
  // params
  nh_private.param("scale", _scale, _scale);
  // publishers
  image_transport::ImageTransport it_private(nh_private);
  _out_pub = it_private.advertise("out", 1);
  // subscribers
  ImgSub in1_sub(it_private, "in1", 1), in2_sub(it_private, "in2", 1);
  message_filters::TimeSynchronizer<Image, Image> sync(in1_sub, in2_sub, 10);
  message_filters::Synchronizer<MySyncPolicy>
               (MySyncPolicy(1), in1_sub, in2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}

