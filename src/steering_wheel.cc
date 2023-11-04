#include <iostream>
#include <Windows.h>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "controller.h"
#include "markers_parser.h"

int main(int argc, char const *argv[])
{
    bool debug = false;
    for( int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-d") {
            debug = true;
        }
    }

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::VideoCapture capture(0);
    cv::Mat frame;
    cv::Mat outputImage;

    if (debug) {
        cv::namedWindow( "debug preview", cv::WindowFlags::WINDOW_AUTOSIZE);
    }
    MarkersParser parser(22, 23, false);
    Steer steer('W', 'S', 'A', 'D', debug);
    while (true) {
        capture >> frame;
        if(frame.empty())
            break;
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
        if (debug) {
            outputImage = frame.clone();
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        }
        WheelPosition position = parser.parse(markerCorners, markerIds);
        if (position.isValid()) {
            cv::Point2f top_center, bottom_center;
            top_center = position.getTop();
            bottom_center = position.getBottom();
            cv::Scalar color = top_center.x - bottom_center.x > 0 ? cv::Scalar(0, 0, 1) * 255 : cv::Scalar(0, 1, 1) * 255;
            cv::Point2f board_center = (top_center + bottom_center) / 2;
            float angle = position.getAngle();
            if (debug) {
                cv::line(outputImage, top_center, bottom_center, color, 5); 
                cv::putText(outputImage, std::to_string(int(std::round(angle))) + " DEG", position.getCenter(), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 3);
            }
            if (angle > 10) {
                steer.right(angle);
                if (debug) 
                    cv::putText(outputImage, "RIGHT", top_center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 0), 3);
            }else if (angle < -10) {
                steer.left(angle);
                if (debug) 
                    cv::putText(outputImage, "LEFT", top_center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 0), 3);
            } else {
                steer.goStraight();
            }
            if (fabs(angle) < 90) {
                steer.forward();
                if (debug) 
                    cv::putText(outputImage, "FORWARD", bottom_center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 0), 3);
            } else {
                steer.backward();
                if (debug) 
                    cv::putText(outputImage, "BACK", bottom_center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 0), 3);
            }

        } else {
            steer.release();
        }

        if (debug) {
            cv::resize(outputImage, outputImage, outputImage.size() );
            cv::imshow("debug preview", outputImage);
            cv::waitKey(20); // waits to display frame
        }
    }
    return 0;
}
