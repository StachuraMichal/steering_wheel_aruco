#define _USE_MATH_DEFINES

#include "markers_parser.h"


cv::Point2f WheelPosition::getCenter() const
{   
    cv::Point2f board_center = (top_.point + bottom_.point) / 2;
    return board_center;
}

float WheelPosition::getDiameter() const
{
     cv::Point2f diameter_vector = (top_.point - bottom_.point);

    return sqrt(diameter_vector.dot(diameter_vector));
}

float WheelPosition::getAngle() const
{
    cv::Point2f center_to_first = top_.point - getCenter();
    float angle = atan2f(-center_to_first.y, center_to_first.x)* 180 / M_PI - 90; // rotate 90 deg to measure angle to y axis -y due to axis in image 
    angle = angle < -180 ? 360 + angle : angle;
    return angle;
}

WheelPosition MarkersParser::reconstruct(const std::vector<cv::Point2f>& corners, const Reconstruction& target_type)
{
    cv::Point2f top_line_center = (corners[0] + corners[3]) / 2;
    cv::Point2f bottom_line_center = (corners[1] + corners[2]) / 2;
    cv::Point2f versor = top_line_center - bottom_line_center;
    int direction = target_type == Reconstruction::TOP ? 1 : -1;
    versor = versor / std::max(sqrt(versor.dot(versor)), 1e-7f) * direction;
    cv::Point2f source_center = (corners[0] + corners[1] + corners[2] + corners[3] ) /4;
    cv::Point2f target_center = source_center + lastPosition_.getDiameter() * versor;
    return WheelPosition(
        {
            .point = target_type == Reconstruction::TOP ? target_center : source_center,
            .is_reconstructed = target_type == Reconstruction::TOP ? true : false
        },
        {
            .point = target_type == Reconstruction::BOTTOM ? target_center : source_center,
            .is_reconstructed = target_type == Reconstruction::BOTTOM ? true : false
        }
    );
}

WheelPosition MarkersParser::parse(MarkersCorners &markersCorners, std::vector<int>& markerIds)
{
    assert(markersCorners.size() == markersCorners.size());
    std::vector<cv::Point2f> top_corners;
    std::vector<cv::Point2f> bottom_corners;

    for (size_t i = 0; i < markersCorners.size(); i++) {
        if (markerIds[i] == top_id_) {
            top_corners = markersCorners[i];
        }
        if (markerIds[i] == bottom_id_) {
            bottom_corners = markersCorners[i];
        }
    }
    if (top_corners.empty() && bottom_corners.empty()) {
        return WheelPosition();
    }

    if (!top_corners.empty() && !bottom_corners.empty()) {
        cv::Point2f top_center = (top_corners[0] + top_corners[1] + top_corners[2] + top_corners[3] ) /4;
        cv::Point2f bottom_center = (bottom_corners[0] + bottom_corners[1] + bottom_corners[2] + bottom_corners[3] ) /4;
        WheelPosition new_position(
            {
                .point = top_center,
                .is_reconstructed = false
            },
            {
                .point = bottom_center,
                .is_reconstructed = false
            }
        );
        lastPosition_ = new_position;
        return new_position;
    }

    if (lastPosition_.isValid()) {
       
        if (!bottom_corners.empty()) {
        
            WheelPosition new_position = reconstruct(bottom_corners, Reconstruction::TOP);
            if(validate(new_position)) {
                cv::Point2f vector = new_position.getTop() - new_position.getBottom();
                for (size_t i = 0; i < bottom_corners.size(); i++) {
                    top_corners.push_back(bottom_corners[i] + vector);
                } 
                markersCorners.push_back(top_corners);
                markerIds.push_back(top_id_);
                return new_position;
            }
        }

        if (!top_corners.empty()) {
        
            WheelPosition new_position = reconstruct(top_corners, Reconstruction::BOTTOM);
            if(validate(new_position)) {
                cv::Point2f vector = new_position.getBottom() - new_position.getTop();
                for (size_t i = 0; i < top_corners.size(); i++) {
                    bottom_corners.push_back(top_corners[i] + vector);
                } 
                markersCorners.push_back(bottom_corners);
                markerIds.push_back(bottom_id_);

                return new_position;
            }
        }
    }
    return WheelPosition();

}

bool MarkersParser::validate(const WheelPosition& new_position)
{
    return precise_mode_ ? fabs(new_position.getAngle() - lastPosition_.getAngle()) < max_angle_diff_ : true;
}
