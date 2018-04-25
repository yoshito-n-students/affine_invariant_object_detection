#ifndef LABEL_DETECTION_LABEL_DRAWER_HPP
#define LABEL_DETECTION_LABEL_DRAWER_HPP

#include <string>
#include <vector>

#include <ros/names.h>
#include <ros/param.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace label_detection {

class LabelDrawer {
public:
  LabelDrawer() {}
  virtual ~LabelDrawer() {}

  void loadParams(const std::string param_ns = "~") {
    namespace rp = ros::param;
    namespace rn = ros::names;

    line_tickness_ = rp::param(rn::append(param_ns, "line_tickness"), 3);
    text_tickness_ = rp::param(rn::append(param_ns, "text_tickness"), 2);
    font_scale_ = rp::param(rn::append(param_ns, "font_scale"), 0.8);
  }

  void draw(cv::Mat &image, const std::vector< std::string > &names,
            const std::vector< std::vector< cv::Point > > &contours) {
    CV_Assert(names.size() == contours.size());

    // dark the original image
    image /= 2;

    // draw contours in red
    cv::polylines(image, contours, true /* is_closed (e.g. draw line from last to first point) */,
                  CV_RGB(255, 0, 0), line_tickness_);

    // draw names in white at the center of corresponding contours
    for (std::size_t i = 0; i < names.size(); ++i) {
      const cv::Rect rect(cv::boundingRect(contours[i]));
      const cv::Size text_size(cv::getTextSize(names[i], cv::FONT_HERSHEY_SIMPLEX, font_scale_,
                                               text_tickness_, NULL /* baseline (won't use) */));
      cv::putText(image, names[i],
                  cv::Point(rect.x + (rect.width - text_size.width) / 2,
                            rect.y + (rect.height + text_size.height) / 2),
                  cv::FONT_HERSHEY_SIMPLEX, font_scale_, CV_RGB(255, 255, 255), text_tickness_);
    }
  }

private:
  int line_tickness_, text_tickness_;
  double font_scale_;
};

} // namespace label_detection

#endif