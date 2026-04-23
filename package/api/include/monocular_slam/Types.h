#ifndef MONOCULAR_SLAM_TYPES_H
#define MONOCULAR_SLAM_TYPES_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace monocular_slam
{

enum class TrackingStatus
{
    SystemNotReady = -1,
    NoImagesYet = 0,
    NotInitialized = 1,
    Ok = 2,
    RecentlyLost = 3,
    Lost = 4,
    OkKlt = 5
};

enum class RunnerState
{
    Idle = 0,
    Running = 1,
    Paused = 2,
    Finished = 3,
    Failed = 4
};

struct KeyPointObservation
{
    float x = 0.0f;
    float y = 0.0f;
    float size = 0.0f;
    float angle = -1.0f;
    float response = 0.0f;
    int octave = 0;
    int class_id = -1;
};

struct Point2D
{
    float x = 0.0f;
    float y = 0.0f;
};

struct Point3D
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct DetectionBox
{
    int class_id = -1;
    float confidence = 0.0f;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    std::string class_name;
};

struct OpticalFlowTrack
{
    Point2D previous_point;
    Point2D current_point;
    bool status = false;
    bool inlier = false;
    float error = 0.0f;
};

struct PointCluster
{
    int id = -1;
    std::vector<Point3D> points;
};

struct MaskPropagationData
{
    bool attempted = false;
    bool succeeded = false;
    std::vector<Point2D> seed_points;
    std::vector<Point2D> flowed_points;
    std::vector<Point3D> tracked_points;
    std::vector<PointCluster> clusters;
};

struct FrameData
{
    std::uint64_t frame_id = 0;
    double timestamp = 0.0;
    int depth_source_frame_id = -1;
    int yolo_source_frame_id = -1;
    int mask_source_frame_id = -1;
    bool has_pose = false;
    bool has_dynamic_mask = false;
    bool has_estimated_depth = false;
    bool used_optical_flow = false;
    bool requested_new_keyframe = false;
    bool is_keyframe = false;
    bool yolo_output_consumed = false;
    bool mask_updated_from_yolo = false;
    bool mask_updated_from_propagation = false;
    bool has_reference_keyframe = false;
    bool tracking_ok = false;
    int tracking_state = -1;
    std::uint64_t reference_keyframe_id = 0;
    TrackingStatus tracking_status = TrackingStatus::SystemNotReady;
    std::string tracking_state_name;
    std::size_t tracked_keypoint_count = 0;
    std::size_t tracked_map_point_count = 0;
    std::size_t active_map_keyframe_count = 0;
    std::size_t active_map_point_count = 0;
    std::array<float, 16> pose_matrix = {1.f, 0.f, 0.f, 0.f,
                                         0.f, 1.f, 0.f, 0.f,
                                         0.f, 0.f, 1.f, 0.f,
                                         0.f, 0.f, 0.f, 1.f};
    cv::Mat frame_bgr;
    cv::Mat dynamic_mask;
    cv::Mat estimated_depth;
    std::vector<KeyPointObservation> raw_orb_keypoints;
    std::vector<KeyPointObservation> filtered_orb_keypoints;
    std::vector<KeyPointObservation> orb_keypoints;
    std::vector<KeyPointObservation> tracked_keypoints;
    std::vector<Point3D> tracked_map_points_world;
    std::vector<DetectionBox> yolo_detections;
    std::vector<OpticalFlowTrack> optical_flow_tracks;
    MaskPropagationData mask_propagation;
};

struct RunMetrics
{
    std::size_t frames_processed = 0;
    double mean_tracking_time_sec = 0.0;
    double median_tracking_time_sec = 0.0;
    bool trajectories_saved = false;
    bool metric_evaluation_attempted = false;
    bool metric_evaluation_succeeded = false;
    int metric_evaluator_exit_code = 0;
    std::string dataset_name;
    std::string trajectory_path;
    std::string keyframe_trajectory_path;
    std::string metrics_csv_path;
};

struct SessionSnapshot
{
    bool initialized = false;
    bool running = false;
    bool paused = false;
    RunnerState runner_state = RunnerState::Idle;
    FrameData current_frame;
    FrameData previous_frame;
    RunMetrics metrics;
};

struct Anchor
{
    std::uint64_t id = 0;
    bool valid = false;
    std::uint64_t source_frame_id = 0;
    double timestamp = 0.0;
    std::string label;
    std::array<float, 16> pose_matrix = {1.f, 0.f, 0.f, 0.f,
                                         0.f, 1.f, 0.f, 0.f,
                                         0.f, 0.f, 1.f, 0.f,
                                         0.f, 0.f, 0.f, 1.f};
};

} // namespace monocular_slam

#endif
