#ifndef LABEL_DETECTION_LABEL_DETECTOR
#define LABEL_DETECTION_LABEL_DETECTOR

#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/names.h>
#include <ros/param.h>

#include <affine_invariant_features/affine_invariant_feature.hpp>
#include <affine_invariant_features/feature_parameters.hpp>
#include <affine_invariant_features/results.hpp>
#include <affine_invariant_features/result_matcher.hpp>
#include <affine_invariant_features/target.hpp>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>

namespace label_detection {

class LabelDetector {
public:
  LabelDetector() {}

  virtual ~LabelDetector() {}

  void loadParams(const std::string ns = "~") {
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
        rp::param< std::string >(rn::append(ns, "reference_directory"), "reference"));
    const std::string parameter_file(
        rp::param< std::string >(rn::append(ns, "parameter_file"), "parameter.yml"));
    const bool use_simple_feature(rp::param(rn::append(ns, "use_simple_feature"), false));
    const double match_stripes(rp::param(rn::append(ns, "match_stripes"), -1.));
    const double match_ratio(rp::param(rn::append(ns, "match_ratio"), 0.05));

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
      if (use_simple_feature) {
        feature_ = params->createFeature();
      } else {
        feature_ = aif::AffineInvariantFeature::create(params->createFeature());
      }
      ROS_INFO_STREAM("Created feature algorithm based on " << params->getDefaultName());
    }

    // set other parameters
    match_stripes_ = match_stripes;
    match_ratio_ = std::min(std::max(match_ratio, 0.), 1.);
  }

  void detect(const cv::Mat &image, std::vector< std::string > &names,
              std::vector< std::vector< cv::Point > > &contours) {
    namespace aif = affine_invariant_features;

    CV_Assert(feature_);
    CV_Assert(image.type() == CV_8UC3);

    // initiate outputs
    names.clear();
    contours.clear();

    // extract features in the image
    aif::Results results;
    feature_->detectAndCompute(image, cv::noArray(), results.keypoints, results.descriptors);

    // match features in the image and the references
    std::vector< cv::Matx33f > transforms;
    std::vector< std::vector< cv::DMatch > > matches_array;
    aif::ResultMatcher::parallelMatch(matchers_, results, transforms, matches_array,
                                      match_stripes_);

    // project matched reference contours
    for (std::size_t i = 0; i < matchers_.size(); ++i) {
      // check the match ratio is enough
      const double nmatched(matches_array[i].size());
      const double nfeatures(matchers_[i]->getReference().keypoints.size());
      if (nmatched / nfeatures < match_ratio_) {
        continue;
      }
      // project contour
      std::vector< cv::Point2f > contour;
      contour.insert(contour.end(), contours_[i].begin(), contours_[i].end());
      cv::perspectiveTransform(contour, contour, transforms[i].inv());
      // push to the outputs
      names.push_back(names_[i]);
      contours.resize(contours.size() + 1);
      contours.back().insert(contours.back().end(), contour.begin(), contour.end());
    }
  }

private:
  std::vector< std::string > names_;
  std::vector< std::vector< cv::Point > > contours_;
  std::vector< cv::Ptr< affine_invariant_features::ResultMatcher > > matchers_;

  cv::Ptr< cv::Feature2D > feature_;

  double match_stripes_;
  double match_ratio_;
};
}

#endif