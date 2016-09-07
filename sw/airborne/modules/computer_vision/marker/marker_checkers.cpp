
using namespace std;
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

#include "marker_checkers.h"

Mat img_marker;
Mat mask;
Mat marker_descriptors;
Ptr<ORB> detector;
Ptr<FlannBasedMatcher> matcher;

float detector_fps = 30; // initial estimate of fps
float detector_fps_epsilon = 0.2; // used to smoothen fps

void init_detect_checkers(void) {

    img_marker = imread("/data/ftp/internal_000/imav/marker_checkers.png", IMREAD_GRAYSCALE);

    detector = ORB::create();
    detector->setMaxFeatures(20);

    std::vector<KeyPoint> keypoints;

    detector->detectAndCompute(img_marker, mask, keypoints, marker_descriptors);
}


struct resultsc opencv_detect_checkers(char *img, int width, int height, int dt) {

    struct resultsc marker;

    // Create new opencv image and convert it to grayscale
    Mat image(height, width, CV_8UC2, img);
    cvtColor(image, image, CV_YUV2GRAY_Y422);

    std::vector<KeyPoint> keypoints;
    Mat descriptors;
    detector->detectAndCompute(image, mask, keypoints, descriptors);
//    drawKeypoints(image, keypoints, image, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    std::vector<DMatch> good;

    if (keypoints.size() > 5) {
        FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 24, 2));
        std::vector<std::vector<DMatch> > matches;
        matcher.knnMatch(descriptors, marker_descriptors, matches, 2);

        for (unsigned int i = 0; i < matches.size(); i++) {
            if (matches[i].size() > 1 and matches[i][0].distance < matches[i][1].distance * 0.75)
                good.push_back(matches[i][0]);
        }
    }

    if (good.size() > 1) {
        marker.detected = true;
        marker.x = 0; marker.y = 0;

        for (unsigned int i = 0; i < good.size(); i++) {
            Point2f point = keypoints[good[i].queryIdx].pt;
            marker.x += point.x;
            marker.y += point.y;
        }

        marker.x /= good.size();
        marker.y /= good.size();
    } else {
        marker.detected = false;
    }

    // Update FPS estimate
    detector_fps = (1 - detector_fps_epsilon) * detector_fps + detector_fps_epsilon * (1000000.f / dt);

    // Draw FPS on image
    char text[50]; sprintf(text,"FPS: %0.2f, M: %i", detector_fps, good.size());
    putText(image, text, Point(10, image.rows-10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 2);

    if (marker.detected) {
        sprintf(text,"x: %i, y: %i", marker.x, marker.y);
        putText(image, text, Point(10, image.rows-20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(150), 2);
    }

    // Convert image back to YUV
    grayscale_opencv_to_yuv422(image, img, width, height);

    return marker;
}