#include <ros/init.h>
#include <ros/node_handle.h>

#include <affine_invariant_object_detection/affine_invariant_object_detection.hpp>

int main(int argc, char *argv[]) {
  namespace aiod = affine_invariant_object_detection;

  ros::init(argc, argv, "affine_invariant_object_detection_node");
  ros::NodeHandle handle;

  aiod::AffineInvariantObjectDetection node(handle);
  node.loadParams();

  ros::spin();

  return 0;
}