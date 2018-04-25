#ifndef LABEL_DETECTION_LABEL_DETECTOR
#define LABEL_DETECTION_LABEL_DETECTOR

#include <algorithm>
#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/names.h>
#include <ros/param.h>

#include <affine_invariant_features/affine_invariant_feature.hpp>
#include <affine_invariant_features/feature_parameters.hpp>
#include <affine_invariant_features/result_matcher.hpp>
#include <affine_invariant_features/results.hpp>
#include <affine_invariant_features/target.hpp>

#include <boost/algorithm/clamp.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace label_detection {

class LabelDetector {
public:
  LabelDetector() {}

  virtual ~LabelDetector() {}

  void loadParams(const std::string param_ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;
    namespace bf = boost::filesystem;
    namespace aif = affine_invariant_features;

    // reset objects
    names_.clear();
    contours_.clear();
    matchers_.clear();

    // load parameters
    const std::string reference_directory(
        rp::param< std::string >(rn::append(param_ns, "reference_directory"), "reference"));
    const std::string parameter_file(
        rp::param< std::string >(rn::append(param_ns, "parameter_file"), "parameter.yml"));
    const double match_stripes(rp::param(rn::append(param_ns, "match_stripes"), -1.));
    const double match_ratio(rp::param(rn::append(param_ns, "match_ratio"), 0.05));
    const double area_ratio(rp::param(rn::append(param_ns, "area_ratio"), 0.1));

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

        // skip an entry not containing a target description with a contour
        const cv::FileStorage file(path.string(), cv::FileStorage::READ);
        const cv::Ptr< const aif::TargetDescription > target(
            aif::load< aif::TargetDescription >(file.root()));
        if (!target) {
          ROS_WARN_STREAM("No target description in " << path << ". Skip.");
          continue;
        }
        if (target->contour.empty()) {
          ROS_WARN_STREAM("No reference contour in " << path << ". Skip.");
          continue;
        }

        // skip an entry not containing valid features
        const cv::Ptr< const aif::Results > reference(aif::load< aif::Results >(file.root()));
        if (!reference) {
          ROS_WARN_STREAM("No reference features in " << path << ". Skip.");
          continue;
        }
        if (reference->keypoints.empty() || reference->descriptors.empty()) {
          ROS_WARN_STREAM("Empty reference feature in " << path << ". Skip.");
          continue;
        }
        if (reference->keypoints.size() != reference->descriptors.rows) {
          ROS_WARN_STREAM("Feature size mismatch in " << path << ". Skip.");
          continue;
        }

        // load valid reference features
        names_.push_back(reference_name);
        contours_.push_back(target->contour);
        matchers_.push_back(new aif::ResultMatcher(reference));
        ROS_INFO_STREAM("Loaded the reference \"" << reference_name << "\" from " << path << " ("
                                                  << reference->keypoints.size() << " features)");
      }
      ROS_INFO_STREAM("Loaded " << names_.size() << " reference feature sets");
    }

    // load feature algorithm
    {
      const cv::FileStorage file(parameter_file, cv::FileStorage::READ);
      cv::Ptr< const aif::FeatureParameters > params(
          aif::load< aif::FeatureParameters >(file.root()));
      if (!params) {
        ROS_WARN_STREAM("No feature parameters in " << parameter_file
                                                    << ". Use default parameters.");
        params = new aif::SIFTParameters();
      }
      feature_ = params->createFeature();
      ROS_INFO_STREAM("Created feature algorithm based on " << params->getDefaultName());
    }

    // set other parameters
    match_stripes_ = match_stripes;
    match_ratio_ = boost::algorithm::clamp(match_ratio, 0., 1.);
    area_ratio_ = boost::algorithm::clamp(area_ratio, 0., 1.);
  }

  void detect(const cv::Mat &image, std::vector< std::string > &names,
              std::vector< std::vector< cv::Point > > &contours) const {
    namespace aif = affine_invariant_features;

    CV_Assert(feature_);
    CV_Assert(image.type() == CV_8UC3);

    // initiate outputs
    names.clear();
    contours.clear();

    // extract features in the image
    aif::Results results;
    feature_->detectAndCompute(image, cv::noArray(), results.keypoints, results.descriptors);
    ROS_INFO_STREAM("Extracted " << results.keypoints.size() << " features from the target image");

    // match features in the image and the references
    std::vector< cv::Matx33f > transforms;
    std::vector< std::vector< cv::DMatch > > matches_array;
    aif::ResultMatcher::parallelMatch(matchers_, results, transforms, matches_array,
                                      std::vector< double >(matchers_.size(), match_ratio_),
                                      match_stripes_);

    // project matched reference contours
    const std::size_t image_area(image.total());
    for (std::size_t i = 0; i < matchers_.size(); ++i) {
      // check the match was succeeded
      if (matches_array[i].empty()) {
        continue;
      }
      // project contour
      std::vector< cv::Point2f > contour;
      contour.insert(contour.end(), contours_[i].begin(), contours_[i].end());
      cv::perspectiveTransform(contour, contour, transforms[i].inv());
      // reject small contour
      if (cv::contourArea(contour) / image_area < area_ratio_) {
        continue;
      }
      // push to the outputs
      names.push_back(names_[i]);
      contours.resize(contours.size() + 1);
      contours.back().insert(contours.back().end(), contour.begin(), contour.end());
    }
  }

private:
  std::vector< std::string > names_;
  std::vector< std::vector< cv::Point > > contours_;
  std::vector< cv::Ptr< const affine_invariant_features::ResultMatcher > > matchers_;

  cv::Ptr< cv::Feature2D > feature_;

  double match_stripes_;
  double match_ratio_;
  double area_ratio_;
};
} // namespace label_detection

#endif