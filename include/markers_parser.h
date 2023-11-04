#pragma once

#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

typedef std::vector<std::vector<cv::Point2f>> MarkersCorners;

struct WheelAnchor
{
    cv::Point2f point;
    bool is_reconstructed;
};


class WheelPosition {
  public:
    cv::Point2f getCenter() const;
    float getDiameter() const;
    float getAngle() const;

    inline cv::Point2f getTop() {
        return top_.point;
    };
    inline cv::Point2f getBottom() {
        return bottom_.point;
    };
  
    inline bool isValid() const {
        return valid_;
    }
    inline void setValid(bool valid) {
        valid_ = valid;
    }

    WheelPosition(const WheelAnchor& top, const WheelAnchor& bottom) :
        top_(top), bottom_(bottom), valid_(true) {};
    WheelPosition() : valid_(false) {};
  private:
    bool valid_;
    WheelAnchor top_;
    WheelAnchor bottom_;
};

class MarkersParser {
  public:
    MarkersParser(int top_id, int bottom_id, bool precise_mode) :
        top_id_(top_id), bottom_id_(bottom_id), precise_mode_(precise_mode) {};
    MarkersParser() = delete;
    MarkersParser(const MarkersParser&) = delete;
    WheelPosition parse(MarkersCorners& markersCorners, std::vector<int>& markerIds);
  private:
    enum class Reconstruction{
      TOP,
      BOTTOM
    };

    WheelPosition reconstruct(const std::vector<cv::Point2f>& corners, const Reconstruction& target_type);  
    bool validate(const WheelPosition& new_position);
    int top_id_;
    int bottom_id_;
    bool precise_mode_;
    float max_angle_diff_ = 10.f;
    WheelPosition lastPosition_;

};