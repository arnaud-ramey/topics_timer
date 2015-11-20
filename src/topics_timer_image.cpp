/*!
  \file       topics_timer_image.cpp
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
  - \b "~timer"
    [std_msgs::Float32]
  - \b "~size_ratio"
    [std_msgs::Float32]
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CompressedImage.h>

inline std::string time2str(const ros::Time & t) {
  std::ostringstream out;
  out << t.sec << '.' << std::setw(9) << std::setfill('0') << t.nsec;
  return out.str();
}

//! Convert a map to a string
template< class MapType >
std::string map_to_string(const MapType & m) {
  std::ostringstream ans_stream;
  for( typename MapType::const_iterator iter = m.begin(), iend = m.end();
       iter != iend; ++iter )
    ans_stream << ":" << iter->first << "->" << iter->second << std::endl;
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////
class TopicsTimerImage {
public:
  typedef sensor_msgs::Image T1;
  typedef sensor_msgs::CompressedImage T2;
  //typedef sensor_msgs::Image T2;

  struct Data {
    Data() : size1(-1), size2(-1) {}
    friend std::ostream & operator << (std::ostream & stream,
                                       const Data & d) {
      stream << '[' << d.time1 << ", " << d.time2 << ']';
      return stream;
    }
    ros::Time time1, time2;
    int size1, size2;
  };
  TopicsTimerImage() : nh_private("~") {
    // create subscribers
    sub1 = nh_private.subscribe("in1", 1, &TopicsTimerImage::image_cb1, this);
    sub2 = nh_private.subscribe("in2", 1, &TopicsTimerImage::image_cb2, this);
    // create publishers
    timer_pub = nh_private.advertise<std_msgs::Float32>("timer", 1);
    size_ratio_pub = nh_private.advertise<std_msgs::Float32>("size_ratio", 1);
    ROS_INFO("TopicsTimerImage: delay between '%s' ans '%s', publishing on '%s'",
             sub1.getTopic().c_str(), sub2.getTopic().c_str(),
             timer_pub.getTopic().c_str());
  }

private:
  void image_cb1(const T1::ConstPtr & msg) {
    ros::Time in1 = ros::Time::now();
    ros::Time stamp = msg->header.stamp;
    Data* data = (&stamp2data[stamp]);
    //data->time1 = in1;
    data->time1 = stamp;
    data->size1 = msg->data.size();
    // printf("stamp2data:%s\n", map_to_string(stamp2data).c_str());
    if (try_publish_data(data))
      stamp2data.erase(stamp);
  }

  void image_cb2(const T2::ConstPtr & msg) {
    ros::Time in2 = ros::Time::now();
    ros::Time stamp = msg->header.stamp;
    Data* data = (&stamp2data[stamp]);
    data->time2 = in2;
    data->size2 = msg->data.size();
    //printf("stamp2data:%s\n", map_to_string(stamp2data).c_str());
    if (try_publish_data(data))
      stamp2data.erase(stamp);
  }


  bool try_publish_data(Data* data) {
    if (data->size1 < 0 || data->size2 < 0)
      return false;
    // time
    std_msgs::Float32 msg_out;
    msg_out.data = (data->time2 - data->time1).toSec();
    timer_pub.publish(msg_out);
    // size_ratio
    msg_out.data = 1. * data->size2 / data->size1;
    size_ratio_pub.publish(msg_out);
    return true;
  }

  std::map<ros::Time, Data> stamp2data;
  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber sub1, sub2;
  ros::Publisher timer_pub, size_ratio_pub;
}; // end class TopicsTimerImage

int main(int argc, char** argv) {
  ros::init(argc, argv, "TopicsTimerImage");
  TopicsTimerImage node;
  ros::spin();
  return 0;
}

