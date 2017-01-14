#ifndef LABEL_DETECTION_LABEL_DETECTION_NODE
#define LABEL_DETECTION_LABEL_DETECTION_NODE

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <ros/console.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <label_detection/label_detector.hpp>

#include <opencv2/core.hpp>

namespace label_detection {

class LabelDetectionNode {
public:
  LabelDetectionNode(const ros::NodeHandle &handle) : handle_(handle) {}

  virtual ~LabelDetectionNode() {}

  void loadParams(const std::string ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;
    namespace it = image_transport;

    // reset objects
    subscriber_.shutdown();
    publisher_.shutdown();

    // load parameters
    const std::string transport(rp::param< std::string >(rn::append(ns, "transport"), "raw"));
    detector_.loadParams(ns);

    // setup communication
    publisher_ = it::ImageTransport(handle_).advertise("image_out", 1, true);
    subscriber_ = it::ImageTransport(handle_).subscribe(
        "image_raw", 1, &LabelDetectionNode::onImageReceived, this, transport);
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;

    // process the received message on demand
    if (publisher_.getNumSubscribers() == 0) {
      return;
    }

    // received message to opencv image
    const cb::CvImagePtr image(cb::toCvCopy(image_msg));
    if (!image) {
      ROS_ERROR("Image conversion error");
      return;
    }
    if (image->image.empty()) {
      ROS_ERROR("Empty image message");
      return;
    }
    if (image->image.type() != CV_8UC3) {
      ROS_ERROR("Not a 24bit color image");
      return;
    }

    // match features in the image and the references
    std::vector< std::string > names;
    std::vector< std::vector< cv::Point > > contours;
    detector_.detect(image->image, names, contours);

    // draw the detection results on the image
    for (std::size_t i = 0; i < names.size(); ++i) {
      static cv::RNG rng;
      // draw the label name here ??
      cv::polylines(image->image, std::vector< std::vector< cv::Point > >(1, contours[i]), true,
                    CV_RGB(rng.uniform(128, 256), rng.uniform(128, 256), rng.uniform(128, 256)), 5);
    }

    // publish the image with the matched contours
    publisher_.publish(image->toImageMsg());
  }

private:
  const ros::NodeHandle handle_;
  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;

  LabelDetector detector_;
};
}

#endif