#ifndef ORB_SLAM3_TELEMETRY_H
#define ORB_SLAM3_TELEMETRY_H

#include <map>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Thirdparty/Sophus/sophus/geometry.hpp"

namespace ORB_SLAM3
{

struct TelemetryPoint2D
{
    float x = 0.0f;
    float y = 0.0f;
};

struct TelemetryPoint3D
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct TelemetryDetection
{
    int class_id = -1;
    float confidence = 0.0f;
    cv::Rect box;
    std::string class_name;
};

struct TelemetryOpticalFlowTrack
{
    TelemetryPoint2D previous_point;
    TelemetryPoint2D current_point;
    bool status = false;
    bool inlier = false;
    float error = 0.0f;
};

struct TelemetryCluster
{
    int id = -1;
    std::vector<TelemetryPoint3D> points;
};

struct MaskPropagationTelemetry
{
    bool attempted = false;
    bool succeeded = false;
    std::vector<TelemetryPoint2D> seed_points;
    std::vector<TelemetryPoint2D> flowed_points;
    std::vector<TelemetryPoint3D> tracked_points;
    std::vector<TelemetryCluster> clusters;
};

struct FrameTelemetry
{
    long unsigned int frame_id = 0;
    double timestamp = 0.0;
    int depth_source_frame_id = -1;
    int yolo_source_frame_id = -1;
    int mask_source_frame_id = -1;
    int tracking_state = -1;
    bool tracking_ok = false;
    bool has_pose = false;
    bool used_optical_flow = false;
    bool requested_new_keyframe = false;
    bool is_keyframe = false;
    bool yolo_output_consumed = false;
    bool mask_updated_from_yolo = false;
    bool mask_updated_from_propagation = false;
    bool has_reference_keyframe = false;
    long unsigned int reference_keyframe_id = 0;
    std::size_t tracked_map_point_count = 0;
    std::size_t active_map_keyframe_count = 0;
    std::size_t active_map_point_count = 0;
    Sophus::SE3f pose;
    cv::Mat dynamic_mask;
    cv::Mat estimated_depth;
    std::vector<cv::KeyPoint> raw_orb_keypoints;
    std::vector<cv::KeyPoint> filtered_orb_keypoints;
    std::vector<cv::KeyPoint> orb_keypoints;
    std::vector<cv::KeyPoint> tracked_keypoints;
    std::vector<TelemetryPoint3D> tracked_map_points_world;
    std::vector<TelemetryDetection> yolo_detections;
    std::vector<TelemetryOpticalFlowTrack> optical_flow_tracks;
    MaskPropagationTelemetry mask_propagation;
};

} // namespace ORB_SLAM3

#endif
