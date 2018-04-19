# label_detection
A ROS package that detects flat objects in an image using affine invariant feature matching

## Nodes
label_detection_node
* online label detection

test_label_detector
* offline label detection test

## Subscribed Topics
(label_detection_node only)

image_raw (sensor_msgs/Image)

## Published Topics
(label_detection_node only)

image_out (sensor_msgs/Image)
* subtopics supported by image_transport are also published

## Parameters
~reference_directory (string, default: "reference")
* path to directory which contains reference feature files (usually <label_name>.yml or <label_name>.yml.gz)

~parameter_file (string, default: "parameter.yml")
* file name which specifies feature detection algorithm (usually <feature_name>.yml)
* must be the same file used in generating reference feature files

~match_stripes (double, default: -1.0)
* number of parallel threads matching features
* -1.0 means as many as possible

~match_ratio (double, default: 0.05)
* highly depends on type of feature detection algorithm
* compared to \<number of matched features>/\<number of all features in reference>
* label is detected if match_ratio is lower

(label_detection_node only)

~image_transport (string, default: "raw")
* transport type of the subscribed image topic
