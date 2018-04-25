#ifndef LABEL_DETECTION_LABEL_DETECTION_NODE
#define LABEL_DETECTION_LABEL_DETECTION_NODE

#include <stdexcept>
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

#include <label_detection/Labels.h>
#include <label_detection/label_detector.hpp>

#include <opencv2/core.hpp>

#include <boost/foreach.hpp>

namespace label_detection {

class LabelDetectionNode {
public:
  LabelDetectionNode(const ros::NodeHandle &nh) : nh_(nh), it_(nh) {}

  virtual ~LabelDetectionNode() {}

  void loadParams(const std::string param_ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;

    // reset objects
    image_subscriber_.shutdown();
    label_publisher_.shutdown();
    image_publisher_.shutdown();

    // load parameters
    desired_encoding_ = rp::param< std::string >(rn::append(param_ns, "desired_encoding"), "bgr8");
    republish_image_ = rp::param(rn::append(param_ns, "republish_image"), false);
    detector_.loadParams(param_ns);

    // setup communication
    if (republish_image_) {
      image_publisher_ = it_.advertise("image_out", 1, true);
    }
    label_publisher_ = nh_.advertise< Labels >("labels_out", 1, true);
    image_subscriber_ = it_.subscribe(
        "image_raw", 1, &LabelDetectionNode::onImageReceived, this,
        image_transport::TransportHints(
            "raw" /* default transport*/, ros::TransportHints() /* message connection hints */,
            ros::NodeHandle(param_ns) /* try load param_ns/image_transport */));
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;

    try {
      // process the received message on demand
      if (image_publisher_.getNumSubscribers() == 0 && label_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // received message to opencv image
      const cb::CvImageConstPtr image(cb::toCvShare(image_msg, desired_encoding_));
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

      // publish the original image for less queue size of subscribers
      // which synchronize processed images and labels
      if (republish_image_) {
        image_publisher_.publish(image_msg);
      }

      // publish names and contours of detected labels
      Labels labels_msg;
      labels_msg.header = image_msg->header;
      labels_msg.names = names;
      BOOST_FOREACH (const std::vector< cv::Point > &points, contours) {
        Points points_msg;
        BOOST_FOREACH (const cv::Point &point, points) {
          Point point_msg;
          point_msg.x = point.x;
          point_msg.y = point.y;
          points_msg.points.push_back(point_msg);
        }
        labels_msg.contours.push_back(points_msg);
      }
      label_publisher_.publish(labels_msg);

    } catch (const std::exception &error) {
      // show runtime error when happened
      ROS_ERROR_STREAM(error.what());
    }
  }

private:
  std::string desired_encoding_;
  bool republish_image_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;

  ros::NodeHandle nh_;
  ros::Publisher label_publisher_;

  LabelDetector detector_;
};

} // namespace label_detection

#endif