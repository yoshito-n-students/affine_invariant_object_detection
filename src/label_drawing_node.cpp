#include <image_transport/subscriber_filter.h>
#include <label_detection/Labels.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <label_detection/label_drawing_node.hpp>

namespace it = image_transport;
namespace ld = label_detection;
namespace mf = message_filters;

void onBothReceived(const sensor_msgs::ImageConstPtr &image, const ld::LabelsConstPtr &labels) {
  ROS_ASSERT(image);
  ROS_ASSERT(labels);
  ROS_ASSERT(image->header.stamp == labels->header.stamp);

  ROS_INFO_STREAM("onBothReceived: " << image->header.stamp);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "label_drawing_node");
  ros::NodeHandle nh;
  it::ImageTransport it(nh);

  it::SubscriberFilter image_subscriber;
  image_subscriber.subscribe(it, "image", 1);
  mf::Subscriber< label_detection::Labels > label_subscriber;
  label_subscriber.subscribe(nh, "labels", 1);

  mf::TimeSynchronizer< sensor_msgs::Image, ld::Labels > both_subscriber(10);
  both_subscriber.connectInput(image_subscriber, label_subscriber);
  both_subscriber.registerCallback(onBothReceived);

  ros::spin();

  return 0;
}