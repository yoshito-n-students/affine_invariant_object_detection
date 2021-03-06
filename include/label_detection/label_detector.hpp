#ifndef LABEL_DETECTION_LABEL_DETECTOR_HPP
#define LABEL_DETECTION_LABEL_DETECTOR_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <label_detection/label_detector_cv.hpp>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>

#include <boost/foreach.hpp>

namespace label_detection {

class LabelDetector : public nodelet::Nodelet {
public:
  LabelDetector() {}

  virtual ~LabelDetector() {}

  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    // init detector
    detector_.init(pnh.param< std::string >("reference_directory", "reference"),
                   pnh.param< std::string >("parameter_file", "parameter.yml"));

    // load parameters
    desired_encoding_ = pnh.param< std::string >("desired_encoding", "bgr8");
    match_ratio_ = pnh.param("match_ratio", 0.05);
    area_ratio_ = pnh.param("area_ratio", 0.1);
    match_stripes_ = pnh.param("match_stripes", -1.);
    republish_image_ = pnh.param("republish_image", false);

    // setup result publishers
    if (republish_image_) {
      image_publisher_ = it.advertise("image_out", 1, true);
    }
    label_publisher_ = nh.advertise< object_detection_msgs::Objects >("labels_out", 1, true);

    // start label detection
    const image_transport::TransportHints default_hints;
    image_subscriber_ = it.subscribe(
        "image_raw", 1, &LabelDetector::onImageReceived, this,
        image_transport::TransportHints(default_hints.getTransport(), default_hints.getRosHints(),
                                        pnh /* try load pnh.resolveName(image_transport) */));
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;
    namespace odm = object_detection_msgs;

    try {
      // process the received message on demand
      if (image_publisher_.getNumSubscribers() == 0 && label_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // received message to opencv image
      const cb::CvImageConstPtr image(cb::toCvShare(image_msg, desired_encoding_));
      if (!image) {
        NODELET_ERROR("Image conversion error");
        return;
      }
      if (image->image.empty()) {
        NODELET_ERROR("Empty image message");
        return;
      }

      // match features in the image and the references
      std::vector< std::string > names;
      std::vector< std::vector< cv::Point > > contours;
      detector_.detect(image->image, names, contours, match_ratio_, area_ratio_, match_stripes_);
      if (names.empty() && contours.empty()) {
        // no labels found
        return;
      }

      // publish the original image for less queue size of subscribers
      // which synchronize processed images and labels
      if (republish_image_) {
        image_publisher_.publish(image_msg);
      }

      // publish names and contours of detected labels
      // (use a shared pointer to avoid data copy between nodelets)
      const odm::ObjectsPtr labels_msg(new odm::Objects);
      labels_msg->header = image_msg->header;
      labels_msg->names = names;
      labels_msg->contours = odm::toContoursMsg(contours);
      label_publisher_.publish(labels_msg);

    } catch (const std::exception &error) {
      // show runtime error when happened
      NODELET_ERROR_STREAM(error.what());
    }
  }

private:
  std::string desired_encoding_;
  double match_ratio_, area_ratio_, match_stripes_;
  bool republish_image_;

  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher label_publisher_;

  LabelDetectorCv detector_;
};

} // namespace label_detection

#endif