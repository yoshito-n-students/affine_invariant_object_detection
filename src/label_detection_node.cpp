#include <ros/init.h>
#include <ros/node_handle.h>

#include <label_detection/label_detection_node.hpp>

int main(int argc, char *argv[]) {
  namespace ld = label_detection;

  ros::init(argc, argv, "label_detection_node");
  ros::NodeHandle handle;

  ld::LabelDetectionNode node(handle);
  node.loadParams();

  ros::spin();

  return 0;
}