#ifndef AFFINE_INVARIANT_OBJECT_DETECTION_AFFINE_INVARIANT_OBJECT_DETECTION
#define AFFINE_INVARIANT_OBJECT_DETECTION_AFFINE_INVARIANT_OBJECT_DETECTION

#include <algorithm>
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

#include <affine_invariant_features/feature_parameters.hpp>
#include <affine_invariant_features/results.hpp>
#include <affine_invariant_features/result_matcher.hpp>
#include <affine_invariant_features/target.hpp>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>

namespace affine_invariant_object_detection {

class AffineInvariantObjectDetection {
public:
  AffineInvariantObjectDetection(const ros::NodeHandle &handle) : handle_(handle) {}

  virtual ~AffineInvariantObjectDetection() {}

  void loadParams(const std::string ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;
    namespace bf = boost::filesystem;
    namespace aif = affine_invariant_features;
    namespace it = image_transport;

    // reset objects
    names_.clear();
    contours_.clear();
    matchers_.clear();
    publisher_.shutdown();
    subscriber_.shutdown();

    // load parameters
    const std::string reference_directory(
        rp::param< std::string >(rn::append(ns, "reference_directory"), "reference"));
    const std::string parameter_file(
        rp::param< std::string >(rn::append(ns, "parameter_file"), "parameter.yml"));
    const double match_ratio(rp::param(rn::append(ns, "match_ratio"), 0.05));
    const std::string transport(rp::param< std::string >(rn::append(ns, "transport"), "raw"));

    // load reference features
    {
      // iterate all entries in the reference directory
      bf::directory_iterator entry(reference_directory);
      bf::directory_iterator entry_end;
      for (; entry != entry_end; ++entry) {

        // skip a directory entry
        const bf::path path(entry->path());
        if (bf::is_directory(path)) {
          ROS_WARN_STREAM("Directory " << path << ". Skip.");
          continue;
        }

        // skip an entry having an invalid filename
        const std::string filename(path.filename().string());
        const std::string reference_name(filename.substr(0, filename.find_first_of('.')));
        if (reference_name.empty()) {
          ROS_WARN_STREAM("No stem in the filename of " << path << ". Skip.");
          continue;
        }
        if (std::find(names_.begin(), names_.end(), reference_name) != names_.end()) {
          ROS_WARN_STREAM("Non-unique stem in the filename of " << path << ". Skip.");
          continue;
        }

        // skip an entry not containing a target description
        const cv::FileStorage file(path.string(), cv::FileStorage::READ);
        aif::TargetDescription target;
        const cv::FileNode target_node(file[target.getDefaultName()]);
        if (target_node.empty()) {
          ROS_WARN_STREAM("No target description in " << path << ". Skip.");
          continue;
        }

        // skip an entry not containing a contour information
        target_node >> target;
        if (target.contour.empty()) {
          ROS_WARN_STREAM("No reference contour in " << path << ". Skip.");
          continue;
        }

        // skip an entry not containing features
        aif::Results reference;
        const cv::FileNode reference_node(file[reference.getDefaultName()]);
        if (reference_node.empty()) {
          ROS_WARN_STREAM("No reference features in " << path << ". Skip.");
          continue;
        }

        // skip an entry containing invalid features
        reference_node >> reference;
        if (reference.keypoints.empty() || reference.descriptors.empty()) {
          ROS_WARN_STREAM("Empty reference feature in " << path << ". Skip.");
          continue;
        }
        if (reference.keypoints.size() != reference.descriptors.rows) {
          ROS_WARN_STREAM("Feature size mismatch in " << path << ". Skip.");
          continue;
        }

        // load valid reference features
        names_.push_back(reference_name);
        contours_.push_back(target.contour);
        matchers_.push_back(new aif::ResultMatcher(reference));
        ROS_INFO_STREAM("Loaded the reference \"" << reference_name << "\" from " << path << " ("
                                                  << reference.keypoints.size() << " features)");
      }
      ROS_INFO_STREAM("Loaded " << names_.size() << " reference feature sets");
    }

    // load feature algorithm
    {
      const cv::FileStorage file(parameter_file, cv::FileStorage::READ);
      cv::Ptr< aif::FeatureParameters > params(aif::readFeatureParameters(file.root()));
      if (!params) {
        ROS_WARN_STREAM("No feature parameters in " << parameter_file
                                                    << ". Use default parameters.");
        params = new aif::SIFTParameters();
      }
      feature_ = params->createFeature();
      ROS_INFO_STREAM("Created feature algorithm based on " << params->getDefaultName());
    }

    // set other parameters
    match_ratio_ = std::min(std::max(match_ratio, 0.), 1.);

    // setup communication
    publisher_ = it::ImageTransport(handle_).advertise("image_out", 1, true);
    subscriber_ = it::ImageTransport(handle_).subscribe(
        "image_raw", 1, &AffineInvariantObjectDetection::onImageReceived, this, transport);
  }

private:
  void onImageReceived(const sensor_msgs::ImageConstPtr &image_msg) {
    namespace cb = cv_bridge;
    namespace aif = affine_invariant_features;

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

    // extract features in the image
    aif::Results results;
    feature_->detectAndCompute(image->image, cv::noArray(), results.keypoints, results.descriptors);

    // match features in the image and the references
    std::vector< cv::Matx33f > transforms;
    std::vector< std::vector< cv::DMatch > > matches_array;
    aif::ResultMatcher::parallelMatch(matchers_, results, transforms, matches_array);

    // project matched reference contours
    std::vector< std::vector< cv::Point > > matched_contours;
    for (std::size_t i = 0; i < matchers_.size(); ++i) {
      const double nmatched(matches_array[i].size());
      const double nfeatures(matchers_[i]->getReference().keypoints.size());
      if (nmatched / nfeatures < match_ratio_) {
        continue;
      }
      std::vector< cv::Point2f > tmp_contour;
      tmp_contour.insert(tmp_contour.end(), contours_[i].begin(), contours_[i].end());
      cv::perspectiveTransform(tmp_contour, tmp_contour, transforms[i]);
      matched_contours.resize(matched_contours.size() + 1);
      matched_contours.back().insert(matched_contours.back().end(), tmp_contour.begin(),
                                     tmp_contour.end());
    }

    // draw the matched contours on the image
    cv::polylines(image->image, matched_contours, true, CV_RGB(255, 0, 0));

    // publish the image with the matched contours
    publisher_.publish(image->toImageMsg());
  }

private:
  const ros::NodeHandle handle_;

  std::vector< std::string > names_;
  std::vector< std::vector< cv::Point > > contours_;
  std::vector< cv::Ptr< affine_invariant_features::ResultMatcher > > matchers_;

  cv::Ptr< cv::Feature2D > feature_;

  double match_ratio_;

  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;
};
}

#endif