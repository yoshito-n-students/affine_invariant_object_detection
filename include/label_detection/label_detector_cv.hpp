#ifndef LABEL_DETECTION_LABEL_DETECTOR_CV_HPP
#define LABEL_DETECTION_LABEL_DETECTOR_CV_HPP

#include <algorithm>
#include <string>
#include <vector>

#include <ros/console.h>

#include <affine_invariant_features/affine_invariant_feature.hpp>
#include <affine_invariant_features/feature_parameters.hpp>
#include <affine_invariant_features/result_matcher.hpp>
#include <affine_invariant_features/results.hpp>
#include <affine_invariant_features/target.hpp>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace label_detection {

class LabelDetectorCv {
public:
  LabelDetectorCv() {}

  virtual ~LabelDetectorCv() {}

  void init(const std::string &reference_directory, const std::string &parameter_file) {
    namespace aif = affine_invariant_features;
    namespace bf = boost::filesystem;

    // reset objects
    names_.clear();
    contours_.clear();
    matchers_.clear();

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
  }

  void detect(const cv::Mat &image, std::vector< std::string > &names,
              std::vector< std::vector< cv::Point > > &contours, const double match_ratio,
              const double area_ratio, const double match_stripes) const {
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
                                      std::vector< double >(matchers_.size(), match_ratio),
                                      match_stripes);

    // project matched reference contours
    const std::size_t image_area(image.total());
    for (std::size_t i = 0; i < matchers_.size(); ++i) {
      // check the match was succeeded
      if (matches_array[i].empty()) {
        continue;
      }
      // project contour
      std::vector< cv::Point > contour;
      transformContour(contours_[i], contour, transforms[i].inv());
      // reject small contour
      if (contourAreaOnImage(contour, image.size()) / image_area < area_ratio) {
        continue;
      }
      // push to the outputs
      names.push_back(names_[i]);
      contours.push_back(contour);
    }
  }

private:
  static void transformContour(const std::vector< cv::Point > &src, std::vector< cv::Point > &dst,
                               const cv::Matx33f &transform) {
    // cv::perspectiveTransform() accepts only array of cv::Point2f
    std::vector< cv::Point2f > contour2f;
    contour2f.assign(src.begin(), src.end());
    cv::perspectiveTransform(contour2f, contour2f, transform);
    dst.assign(contour2f.begin(), contour2f.end());
  }

  static double contourAreaOnImage(const std::vector< cv::Point > &contour,
                                   const cv::Size &image_size) {
    // count number of pixels both in image frame and contour area
    cv::Mat image_frame(cv::Mat::zeros(image_size, CV_8UC1));
    cv::fillPoly(image_frame, std::vector< std::vector< cv::Point > >(1, contour), 255);
    // return value is double (consistency with cv::contourArea())
    return cv::countNonZero(image_frame);
  }

private:
  // reference data
  std::vector< std::string > names_;
  std::vector< std::vector< cv::Point > > contours_;
  std::vector< cv::Ptr< const affine_invariant_features::ResultMatcher > > matchers_;

  // feature algorithm for incomming images
  cv::Ptr< cv::Feature2D > feature_;
};

} // namespace label_detection

#endif