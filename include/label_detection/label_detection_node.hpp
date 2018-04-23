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
    desired_encoding_ = rp::param< std::string >(rn::append(param_ns, "desired_encoding"), "bgr8");
    line_tickness_ = rp::param(rn::append(param_ns, "line_tickness"), 3);
    text_tickness_ = rp::param(rn::append(param_ns, "text_tickness"), 2);
    font_scale_ = rp::param(rn::append(param_ns, "font_scale"), 0.8);
    detector_.loadParams(param_ns);

    // setup communication
    publisher_ = it::ImageTransport(nh_).advertise("image_out", 1, true);
    subscriber_ = it::ImageTransport(nh_).subscribe(
        "image_raw", 1, &LabelDetectionNode::onImageReceived, this,
        it::TransportHints("raw" /* default transport*/,
                           ros::TransportHints() /* message connection hints */,
                           ros::NodeHandle(param_ns) /* try load param_ns/image_transport */));
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;

    try {
      // process the received message on demand
      if (publisher_.getNumSubscribers() == 0) {
        return;
      }

      // received message to opencv image
      const cb::CvImagePtr image(cb::toCvCopy(image_msg, desired_encoding_));
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

      // TODO:
      //   - publish not annotated image but names and contours of detected labels
      //     so that user can use the detection results in their favorite way
      //   - move drawing codes below to a new node

      // draw the detection results on the image
      image->image /= 2;
      for (std::size_t i = 0; i < contours.size(); ++i) {
        // draw the contours in red
        cv::polylines(image->image, std::vector< std::vector< cv::Point > >(1, contours[i]),
                      true /* is_closed (e.g. draw line from last to first) */, CV_RGB(255, 0, 0),
                      line_tickness_);
        // draw the name in white at the center of contours
        const cv::Rect rect(cv::boundingRect(contours[i]));
        const cv::Size text_size(cv::getTextSize(names[i], cv::FONT_HERSHEY_SIMPLEX, font_scale_,
                                                 text_tickness_, NULL /* baseline (won't use) */));
        cv::putText(image->image, names[i],
                    cv::Point(rect.x + (rect.width - text_size.width) / 2,
                              rect.y + (rect.height + text_size.height) / 2),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale_, CV_RGB(255, 255, 255), text_tickness_);
      }

      // publish the image with the matched contours
      publisher_.publish(image->toImageMsg());

    } catch (const std::exception &error) {
      // show runtime error when happened
      ROS_ERROR_STREAM(error.what());
    }
  }

private:
  std::string desired_encoding_;
  int line_tickness_, text_tickness_;
  double font_scale_;

  const ros::NodeHandle nh_;
  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;

  LabelDetector detector_;
};

} // namespace label_detection

#endif