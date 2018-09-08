/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh, std::string cameraTopicName, std::string cameraDepthTopicName)
    : nodeHandle_(nh),
      imageTransport_(nodeHandle_),
      numClasses_(0),
      classLabels_(0),
      rosBoxes_(0),
      rosBoxCounter_(0),
      imageSubscriber_(imageTransport_, cameraTopicName, 1),
      imageDepthSubscriber_(imageTransport_, cameraDepthTopicName, 1),
      sync(MySyncPolicy(10), imageSubscriber_, imageDepthSubscriber_)
{
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Load common parameters.
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                    std::string("tiny-yolo-voc.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("tiny-yolo-voc.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_,
                0, 0, 1, 0.5, 0, 0, 0, 0);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string cameraDepthTopicName;
  std::string cameraSyncTopicName;
  std::string cameraDepthSyncTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxes2DTopicName;
  int boundingBoxes2DQueueSize;
  bool boundingBoxes2DLatch;
  std::string boundingBoxes3DTopicName;
  int boundingBoxes3DQueueSize;
  bool boundingBoxes3DLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;
  int rate;
  syncImage = false;
  publishSyncEnable = false;

  nodeHandle_.param("frame_rate", rate, 2);
  run_period = std::chrono::duration<double>(1.0/rate);

  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("subscribers/camera_depth_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/camera_sync/rgb_topic", cameraSyncTopicName,
                    std::string("/sync/rgb/image"));
  nodeHandle_.param("publishers/camera_sync/depth_topic", cameraDepthSyncTopicName,
                    std::string("/sync/depth/image"));
  nodeHandle_.param("publishers/camera_sync/enable", publishSyncEnable, false);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName,
                    std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes2D/topic", boundingBoxes2DTopicName,
                    std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes2D/queue_size", boundingBoxes2DQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes_2d/latch", boundingBoxes2DLatch, false);
  nodeHandle_.param("publishers/bounding_boxes3D/topic", boundingBoxes3DTopicName,
                    std::string("bounding_boxes_3d"));
  nodeHandle_.param("publishers/bounding_boxes3D/queue_size", boundingBoxes3DQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes3D/latch", boundingBoxes3DLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);



  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

// https://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
  sync.registerCallback( boost::bind(&YoloObjectDetector::cameraCallback, this, _1, _2) );

  objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName,
                                                           objectDetectorQueueSize,
                                                           objectDetectorLatch);
  boundingBoxes2DPublisher_ = nodeHandle_.advertise<sara_msgs::BoundingBoxes2D>(
          boundingBoxes2DTopicName, boundingBoxes2DQueueSize, boundingBoxes2DLatch);
  boundingBoxes3DPublisher_ = nodeHandle_.advertise<sara_msgs::BoundingBoxes3D>(
          boundingBoxes3DTopicName, boundingBoxes2DQueueSize, boundingBoxes3DLatch);
  frameToBoxClient = nodeHandle_.serviceClient<wm_frame_to_box::GetBoundingBoxes3D>("/get_3d_bounding_boxes");


  rgbPublisher_ = imageTransport_.advertise(cameraSyncTopicName, 5);
  depthPublisher_ = imageTransport_.advertise(cameraDepthSyncTopicName, 5);
  detectionImagePublisher_ = imageTransport_.advertise(detectionImageTopicName,
                                                         detectionImageQueueSize,
                                                         detectionImageLatch);
  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName,
                    std::string("check_for_objects"));
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& rgbImageMsg, const sensor_msgs::ImageConstPtr& depthImageMsg)
{
  ROS_DEBUG("[YoloObjectDetector] USB image received.");
  if (!syncImage) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      if (rgbImage2 != nullptr) {
        rgbImage = std::move(rgbImage2);
        depthImage = std::move(depthImage2);
      } else {
        rgbImage = std::move(rgbImageMsg);
        depthImage = std::move(depthImageMsg);
      }

      rgbImage2 = std::move(rgbImageMsg);
      depthImage2 = std::move(depthImageMsg);
      syncImage = true;
    }
  }
  std::cout << "reception";

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(rgbImageMsg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
{
  if (detectionImagePublisher_.getNumSubscribers() < 1)
    return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

double YoloObjectDetector::getWallTime()
{
  struct timeval time;
  if (gettimeofday(&time, NULL)) {
    return 0;
  }
  return (double) time.tv_sec + (double) time.tv_usec * .000001;
}

void *YoloObjectDetector::fetchInThread()
{
  IplImage* ROS_img = getIplImage();
  ipl_into_image(ROS_img, buff_[buffIndex_]);
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    buffId_[buffIndex_] = actionId_;
  }
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_.w, net_.h, buffLetter_[buffIndex_]);
  return 0;
}

void *YoloObjectDetector::detectInThread()
{
  running_ = 1;
  float nms = .4;

  layer l = net_.layers[net_.n - 1];
  float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float *prediction = network_predict(net_, X);

  memcpy(predictions_[demoIndex_], prediction, l.outputs * sizeof(float));
  mean_arrays(predictions_, demoFrame_, l.outputs, avg_);
  l.output = lastAvg2_;
  if (demoDelay_ == 0)
    l.output = avg_;
  if (l.type == DETECTION) {
    get_detection_boxes(l, 1, 1, demoThresh_, probs_, boxes_, 0);
  } else if (l.type == REGION) {
    get_region_boxes(l, buff_[0].w, buff_[0].h, net_.w, net_.h, demoThresh_, probs_, boxes_, 0, 0,
                     demoHier_, 1);
  } else {
    error("Last layer must produce detections\n");
  }
  if (nms > 0)
    do_nms_obj(boxes_, probs_, l.w * l.h * l.n, l.classes, nms);

  if (enableConsoleOutput_) {
    printf("\nFPS:%.1f\n", fps_);
    printf("Objects:\n\n");
  }
  image display = buff_[(buffIndex_ + 2) % 3];
  draw_detections(display, demoDetections_, demoThresh_, boxes_, probs_, demoNames_, demoAlphabet_,
                  demoClasses_);

  // extract the bounding boxes and send them to ROS
  int total = l.w * l.h * l.n;
  int i, j;
  int count = 0;
  for (i = 0; i < total; ++i) {
    float xmin = boxes_[i].x - boxes_[i].w / 2.;
    float xmax = boxes_[i].x + boxes_[i].w / 2.;
    float ymin = boxes_[i].y - boxes_[i].h / 2.;
    float ymax = boxes_[i].y + boxes_[i].h / 2.;

    if (xmin < 0)
      xmin = 0;
    if (ymin < 0)
      ymin = 0;
    if (xmax > 1)
      xmax = 1;
    if (ymax > 1)
      ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < l.classes; ++j) {
      if (probs_[i][j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = probs_[i][j];
          count++;
        }
      }
    }
    syncImage = false;
  }

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) {
    roiBoxes_[0].num = 0;
  } else {
    roiBoxes_[0].num = count;
  }

  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

void *YoloObjectDetector::displayInThread(void *ptr)
{
  show_image_cv(buff_[(buffIndex_ + 1) % 3], "Demo", ipl_);
  int c = cvWaitKey(waitKeyDelay_);
  if (c != -1)
    c = c % 256;
  if (c == 10) {
    if (demoDelay_ == 0)
      demoDelay_ = 60;
    else if (demoDelay_ == 5)
      demoDelay_ = 0;
    else if (demoDelay_ == 60)
      demoDelay_ = 5;
    else
      demoDelay_ = 0;
  } else if (c == 27) {
    demoDone_ = 1;
    return 0;
  } else if (c == 82) {
    demoThresh_ += .02;
  } else if (c == 84) {
    demoThresh_ -= .02;
    if (demoThresh_ <= .02)
      demoThresh_ = .02;
  } else if (c == 83) {
    demoHier_ += .02;
  } else if (c == 81) {
    demoHier_ -= .02;
    if (demoHier_ <= .0)
      demoHier_ = .0;
  }
  return 0;
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                      int frames, int fullscreen)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
  image **alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO_V2\n");
  net_ = parse_network_cfg(cfgfile);
  if (weightfile) {
    load_weights(&net_, weightfile);
  }
  set_batch_network(&net_, 1);
}

void YoloObjectDetector::yolo()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!getImageStatus()) {
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  layer l = net_.layers[net_.n - 1];
  demoDetections_ = l.n * l.w * l.h;
  int j;

  avg_ = (float *) calloc(l.outputs, sizeof(float));
  lastAvg_ = (float *) calloc(l.outputs, sizeof(float));
  lastAvg2_ = (float *) calloc(l.outputs, sizeof(float));
  for (j = 0; j < demoFrame_; ++j)
    predictions_[j] = (float *) calloc(l.outputs, sizeof(float));

  boxes_ = (box *) calloc(l.w * l.h * l.n, sizeof(box));
  roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));
  probs_ = (float **) calloc(l.w * l.h * l.n, sizeof(float *));
  for (j = 0; j < l.w * l.h * l.n; ++j)
    probs_[j] = (float *) calloc(l.classes + 1, sizeof(float));

  IplImage* ROS_img = getIplImage();
  buff_[0] = ipl_to_image(ROS_img);
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  buffLetter_[0] = letterbox_image(buff_[0], net_.w, net_.h);
  buffLetter_[1] = letterbox_image(buff_[0], net_.w, net_.h);
  buffLetter_[2] = letterbox_image(buff_[0], net_.w, net_.h);
  ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

  int count = 0;

  if (!demoPrefix_ && viewImage_) {
    cvNamedWindow("Demo", CV_WINDOW_NORMAL);
    if (fullScreen_) {
      cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
      cvMoveWindow("Demo", 0, 0);
      cvResizeWindow("Demo", 640, 480);
    }
  }

  demoTime_ = getWallTime();

  while (!demoDone_) {
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);


    if (!demoPrefix_) {
      if (count % (demoDelay_ + 1) == 0) {
        fps_ = 1. / (getWallTime() - demoTime_);
        demoTime_ = getWallTime();
        float *swap = lastAvg_;
        lastAvg_ = lastAvg2_;
        lastAvg2_ = swap;
        memcpy(lastAvg_, avg_, l.outputs * sizeof(float));
      }
      if (viewImage_) {
        displayInThread(0);
      }
      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }


    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = end-start;

    if (run_period > elapsed_time)
      std::this_thread::sleep_for(run_period - elapsed_time);  // Sleep a bit so you don't kill my GPU!
  }

}

IplImage* YoloObjectDetector::getIplImage()
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
  IplImage* ROS_img = new IplImage(camImageCopy_);
  return ROS_img;
}

bool YoloObjectDetector::getImageStatus(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void *YoloObjectDetector::publishInThread()
{


  // Publish image.
  cv::Mat cvImage = cv::cvarrToMat(ipl_);
  if (!publishDetectionImage(cv::Mat(cvImage))) {
    ROS_DEBUG("Detection image has not been broadcasted.");
  }

  // Publish bounding boxes and detection result.
  int num = roiBoxes_[0].num;
  if (num > 0 && num <= 100) {
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (roiBoxes_[i].Class == j) {
          rosBoxes_[j].push_back(roiBoxes_[i]);
          rosBoxCounter_[j]++;
        }
      }
    }

    std_msgs::Int8 msg;
    msg.data = num;
    objectPublisher_.publish(msg);

    // Fill the 2D bounding boxes list
    frameToBoxForm.request.boundingBoxes2D.header = depthImage->header;
    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) {
        sara_msgs::BoundingBox2D boundingBox2D;
        sara_msgs::BoundingBox3D boundingBox3D;

        for (int j = 0; j < rosBoxCounter_[i]; j++) {
          auto xmin = int((rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_);
          auto ymin = int((rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_);
          auto xmax = int((rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_);
          auto ymax = int((rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_);

          // Filling 2D bounding boxes
          boundingBox2D.Class = classLabels_[i];
          boundingBox2D.probability = rosBoxes_[i][j].prob;
          boundingBox2D.xmin = xmin;
          boundingBox2D.ymin = ymin;
          boundingBox2D.xmax = xmax;
          boundingBox2D.ymax = ymax;
          frameToBoxForm.request.boundingBoxes2D.boundingBoxes.push_back(boundingBox2D);


        }
      }
    }

    // Fill the 3D bounding boxes list
    frameToBoxClient.waitForExistence();
    frameToBoxForm.request.image = *depthImage;
    frameToBoxForm.request.input_frame = depthImage->header.frame_id;
    frameToBoxForm.request.output_frame = depthImage->header.frame_id;
    frameToBoxClient.call(frameToBoxForm);

    // Publish all of the bounding boxes
    boundingBoxes2DPublisher_.publish(frameToBoxForm.request.boundingBoxes2D);
    boundingBoxes3DPublisher_.publish(frameToBoxForm.response.boundingBoxes3D);

  } else {
    std_msgs::Int8 msg;
    msg.data = 0;
    objectPublisher_.publish(msg);
  }
  frameToBoxForm.request.boundingBoxes2D.boundingBoxes.clear();
  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  return 0;
}


} /* namespace darknet_ros*/
