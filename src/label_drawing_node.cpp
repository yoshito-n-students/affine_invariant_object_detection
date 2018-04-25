#include <ros/init.h>
#include <ros/node_handle.h>

#include <label_detection/label_drawing_node.hpp>

int main(int argc, char *argv[]) {
  namespace ld = label_detection;

  ros::init(argc, argv, "label_drawing_node");
  ros::NodeHandle nh;

  ld::LabelDrawingNode node(nh);

  ros::spin();

  return 0;
}