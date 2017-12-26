#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <label_detection/label_detector.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char *argv[]) {
  namespace ld = label_detection;
  namespace rp = ros::param;

  ros::init(argc, argv, "test_label_detector");
  ros::NodeHandle handle;

  cv::CommandLineParser args(argc, argv,
                             "{ help | | }"
                             "{ @src_image | <none> | }"
                             "{ @dst_image | | optional output image }");

  if (args.has("help")) {
    args.printMessage();
    return 0;
  }

  const std::string src_path(args.get< std::string >("@src_image"));
  const std::string dst_path(args.get< std::string >("@dst_image"));
  if (!args.check()) {
    args.printErrors();
    return 1;
  }

  const cv::Mat src_image(cv::imread(src_path));
  if (src_image.empty()) {
    return 1;
  }

  ld::LabelDetector detector;
  detector.loadParams();
  const int line_tickness(rp::param("~line_tickness", 3));
  const double font_scale(rp::param("~font_scale", 0.8));
  const int text_tickness(rp::param("~text_tickness", 2));

  std::vector< std::string > names;
  std::vector< std::vector< cv::Point > > contours;
  const ros::Time start_time(ros::Time::now());
  detector.detect(src_image, names, contours);
  const ros::Time end_time(ros::Time::now());
  ROS_INFO_STREAM((end_time - start_time).toSec() << "s elapsed to detect");

  cv::Mat dst_image(src_image / 2);
  for (std::size_t i = 0; i < contours.size(); ++i) {
    //
    cv::polylines(dst_image, std::vector< std::vector< cv::Point > >(1, contours[i]), true,
                  CV_RGB(255, 0, 0), line_tickness);
    //
    const cv::Rect rect(cv::boundingRect(contours[i]));
    const cv::Size text_size(
        cv::getTextSize(names[i], cv::FONT_HERSHEY_SIMPLEX, font_scale, text_tickness, NULL));
    cv::putText(dst_image, names[i],
                cv::Point(rect.x + (rect.width - text_size.width) / 2,
                          rect.y + (rect.height + text_size.height) / 2),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, CV_RGB(255, 255, 255), text_tickness);
  }

  cv::imshow("dst_image", dst_image);
  cv::waitKey(0);

  if (!dst_path.empty()) {
    cv::imwrite(dst_path, dst_image);
  }

  return 0;
}