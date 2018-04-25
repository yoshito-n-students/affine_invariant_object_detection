#ifndef LABEL_DETECTION_LABEL_DRAWING_NODE_HPP
#define LABEL_DETECTION_LABEL_DRAWING_NODE_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/console.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <sensor_msgs/Image.h>

#include <label_detection/Labels.h>
#include <label_detection/label_drawer.hpp>

#include <opencv2/core.hpp>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

namespace label_detection {

class LabelDrawingNode {
private:
  typedef message_filters::TimeSynchronizer< sensor_msgs::Image, Labels > SyncSubscriber;

public:
  LabelDrawingNode(const ros::NodeHandle &nh) : nh_(nh), it_(nh) {}
  virtual ~LabelDrawingNode() {}

  void loadParams(const std::string &param_ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;

    // reset objects
    sync_subscriber_.reset();
    label_subscriber_.unsubscribe();
    image_subscriber_.unsubscribe();
    image_publisher_.shutdown();

    // load params
    const int queue_size(rp::param(rn::append(param_ns, "queue_size"), 10));
    drawer_.loadParams(param_ns);

    // annotated image publisher
    image_publisher_ = it_.advertise("image_out", 1, true);

    // detection result subscribers
    image_subscriber_.subscribe(it_, "image_in", 1);
    label_subscriber_.subscribe(nh_, "labels_in", 1);

    // callback on synchronized results
    sync_subscriber_.reset(new SyncSubscriber(queue_size));
    sync_subscriber_->connectInput(image_subscriber_, label_subscriber_);
    sync_subscriber_->registerCallback(&LabelDrawingNode::onSynchronized, this);
  }

private:
  void onSynchronized(const sensor_msgs::ImageConstPtr &image_msg,
                      const LabelsConstPtr &labels_msg) {
    namespace cb = cv_bridge;

    try {
      // process the received message on demand
      if (image_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // received message to opencv image
      // (desired encoding of opencv's drawering functions is bgr8)
      const cb::CvImagePtr image(cb::toCvCopy(image_msg, "bgr8"));
      if (!image) {
        ROS_ERROR("Image conversion error");
        return;
      }
      if (image->image.empty()) {
        ROS_ERROR("Empty image message");
        return;
      }

      // extract contours from message
      std::vector< std::vector< cv::Point > > contours;
      BOOST_FOREACH (const Points &points_msg, labels_msg->contours) {
        std::vector< cv::Point > points;
        BOOST_FOREACH (const Point &point_msg, points_msg.points) {
          points.push_back(cv::Point(point_msg.x, point_msg.y));
        }
        contours.push_back(points);
      }

      // draw labels on image
      drawer_.draw(image->image, labels_msg->names, contours);

      // publish annotated image
      image_publisher_.publish(image->toImageMsg());

    } catch (const std::exception &error) {
      // show runtime error when happened
      ROS_ERROR_STREAM(error.what());
    }
  }

private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter image_subscriber_;
  image_transport::Publisher image_publisher_;

  ros::NodeHandle nh_;
  message_filters::Subscriber< Labels > label_subscriber_;

  boost::scoped_ptr< SyncSubscriber > sync_subscriber_;

  LabelDrawer drawer_;
};

} // namespace label_detection

#endif