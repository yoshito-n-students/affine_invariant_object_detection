#ifndef LABEL_DETECTION_LABEL_DETECTION_NODE
#define LABEL_DETECTION_LABEL_DETECTION_NODE

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <ros/console.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/transport_hints.h>

#include <label_detection/label_detector.hpp>

#include <opencv2/core.hpp>

namespace label_detection {

class LabelDetectionNode {
public:
  LabelDetectionNode(const ros::NodeHandle &nh) : nh_(nh) {}

  virtual ~LabelDetectionNode() {}

  void loadParams(const std::string param_ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;
    namespace it = image_transport;

    // reset objects
    subscriber_.shutdown();
    publisher_.shutdown();

    // load parameters
    detector_.loadParams(param_ns);

    // setup communication
    publisher_ = it::ImageTransport(nh_).advertise("image_out", 1, true);
    subscriber_ = it::ImageTransport(nh_).subscribe(
        "image_raw", 1, &LabelDetectionNode::onImageReceived, this,
        it::TransportHints("raw", ros::TransportHints(), ros::NodeHandle(param_ns)));
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;

    // process the received message on demand
    if (publisher_.getNumSubscribers() == 0) {
      return;
    }

    // received message to opencv image
    const cb::CvImagePtr image(cb::toCvCopy(image_msg, "bgr8"));
    if (!image) {
      ROS_ERROR("Image conversion error");
      return;
    }
    if (image->image.empty()) {
      ROS_ERROR("Empty image message");
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
  const ros::NodeHandle nh_;
  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;

  LabelDetector detector_;
};

} // namespace label_detection

#endif