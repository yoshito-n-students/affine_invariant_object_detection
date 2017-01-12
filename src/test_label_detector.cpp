#include <string>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <label_detection/label_detector.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char *argv[]) {
  namespace ld = label_detection;

  ros::init(argc, argv, "test_label_detector");
  ros::NodeHandle handle;

  cv::CommandLineParser args(argc, argv, "{ help | | }"
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

  std::vector< std::string > names;
  std::vector< std::vector< cv::Point > > contours;
  detector.detect(src_image, names, contours);

  cv::Mat dst_image(src_image.clone());
  {
    cv::RNG rng;
    for (std::size_t i = 0; i < contours.size(); ++i) {
      cv::polylines(dst_image, std::vector< std::vector< cv::Point > >(1, contours[i]), true,
                    CV_RGB(rng.uniform(128, 256), rng.uniform(128, 256), rng.uniform(128, 256)), 5);
    }
  }

  cv::imshow("dst_image", dst_image);
  cv::waitKey(0);

  if (!dst_path.empty()) {
    cv::imwrite(dst_path, dst_image);
  }

  return 0;
}