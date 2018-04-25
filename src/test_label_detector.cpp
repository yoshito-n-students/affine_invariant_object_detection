#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <label_detection/label_detector.hpp>
#include <label_detection/label_drawer.hpp>

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
  ld::LabelDrawer drawer;
  detector.loadParams();
  drawer.loadParams();

  std::vector< std::string > names;
  std::vector< std::vector< cv::Point > > contours;
  const ros::Time start_time(ros::Time::now());
  detector.detect(src_image, names, contours);
  const ros::Time end_time(ros::Time::now());
  ROS_INFO_STREAM((end_time - start_time).toSec() << "s elapsed to detect");

  cv::Mat dst_image(src_image.clone());
  drawer.draw(dst_image,names,contours);

  cv::imshow("dst_image", dst_image);
  cv::waitKey(0);

  if (!dst_path.empty()) {
    cv::imwrite(dst_path, dst_image);
  }

  return 0;
}