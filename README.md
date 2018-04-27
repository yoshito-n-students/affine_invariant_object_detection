# label_detection
A ROS package that detects flat objects in an image using affine invariant feature matching

## Dependencies
affine_invariant_features
* https://github.com/yoshito-n-students/affine_invariant_features

object_detection_msgs
* https://github.com/yoshito-n-students/object_detection_msgs

## Detection Nodes
label_detection_node
* online label detection

test_label_detector
* offline label detection test

## Subscribed Topics
(label_detection_node only)

image_raw (sensor_msgs/Image)

## Published Topics
(label_detection_node only)

labels_out (object_detection_msgs/Objects)

image_out (sensor_msgs/Image)
* original image on which labels are detected
* advertised and published when ~republish_image is true
* subtopics supported by image_transport are also published

## Parameters
~reference_directory (string, default: "reference")
* path to directory which contains reference feature files (usually <label_name>.yml or <label_name>.yml.gz)

~parameter_file (string, default: "parameter.yml")
* path to file which specifies feature detection algorithm (usually <feature_name>.yml)
* must be the same file used in generating reference feature files

~match_stripes (double, default: -1.0)
* number of parallel threads matching features
* -1.0 means as many as possible

~match_ratio (double, default: 0.05)
* highly depends on type of feature detection algorithm
* compared to \<number of matched features>/\<number of all features in reference>
* label is detected if ~match_ratio is lower

~area_ratio (double, default:0.1)
* compared to \<area of label>/\<total area of image>
* label is detected if ~area_ratio is lower

(label_detection_node only)

~desired_encoding (string, default: "bgr8")
* desired image encoding for internal processing
* the encoding of an incomming image will be converted to ~desired_encoding
* must be the same as the encoding of reference images

~republish_image (bool, default: false)
* republish an image when labels on it are detected
* useful for smaller ~queue_size of external time synchronizer for image and label messages

~image_transport (string, default: "raw")
* transport type of the subscribed image topic
