#include "monocular_slam/MonocularPipeline.h"

#include <mutex>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#include "System.h"

namespace monocular_slam
{

namespace
{

std::array<float, 16> ToMatrixArray(const Sophus::SE3f& pose)
{
    std::array<float, 16> out{};
    Eigen::Matrix4f matrix = pose.matrix();
    for(int r = 0; r < 4; ++r)
        for(int c = 0; c < 4; ++c)
            out[static_cast<size_t>(r * 4 + c)] = matrix(r, c);
    return out;
}

cv::Mat ToBgrFrame(const cv::Mat& frame)
{
    if(frame.empty())
        return cv::Mat();

    if(frame.channels() == 3)
        return frame.clone();

    cv::Mat bgr;
    if(frame.channels() == 1)
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    else if(frame.channels() == 4)
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    else
        bgr = frame.clone();
    return bgr;
}

TrackingStatus ToTrackingStatus(const int state)
{
    switch(state)
    {
        case ORB_SLAM3::Tracking::NO_IMAGES_YET: return TrackingStatus::NoImagesYet;
        case ORB_SLAM3::Tracking::NOT_INITIALIZED: return TrackingStatus::NotInitialized;
        case ORB_SLAM3::Tracking::OK: return TrackingStatus::Ok;
        case ORB_SLAM3::Tracking::RECENTLY_LOST: return TrackingStatus::RecentlyLost;
        case ORB_SLAM3::Tracking::LOST: return TrackingStatus::Lost;
        case ORB_SLAM3::Tracking::OK_KLT: return TrackingStatus::OkKlt;
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
        default:
            return TrackingStatus::SystemNotReady;
    }
}

std::string TrackingStateName(const int state)
{
    switch(state)
    {
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: return "SYSTEM_NOT_READY";
        case ORB_SLAM3::Tracking::NO_IMAGES_YET: return "NO_IMAGES_YET";
        case ORB_SLAM3::Tracking::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case ORB_SLAM3::Tracking::OK: return "OK";
        case ORB_SLAM3::Tracking::RECENTLY_LOST: return "RECENTLY_LOST";
        case ORB_SLAM3::Tracking::LOST: return "LOST";
        case ORB_SLAM3::Tracking::OK_KLT: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

KeyPointObservation ToKeyPoint(const cv::KeyPoint& kp)
{
    KeyPointObservation out;
    out.x = kp.pt.x;
    out.y = kp.pt.y;
    out.size = kp.size;
    out.angle = kp.angle;
    out.response = kp.response;
    out.octave = kp.octave;
    out.class_id = kp.class_id;
    return out;
}

Point2D ToPoint2D(const ORB_SLAM3::TelemetryPoint2D& point)
{
    Point2D out;
    out.x = point.x;
    out.y = point.y;
    return out;
}

Point3D ToPoint3D(const ORB_SLAM3::TelemetryPoint3D& point)
{
    Point3D out;
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    return out;
}

DetectionBox ToDetectionBox(const ORB_SLAM3::TelemetryDetection& detection)
{
    DetectionBox out;
    out.class_id = detection.class_id;
    out.confidence = detection.confidence;
    out.x = detection.box.x;
    out.y = detection.box.y;
    out.width = detection.box.width;
    out.height = detection.box.height;
    out.class_name = detection.class_name;
    return out;
}

OpticalFlowTrack ToOpticalFlowTrack(const ORB_SLAM3::TelemetryOpticalFlowTrack& track)
{
    OpticalFlowTrack out;
    out.previous_point = ToPoint2D(track.previous_point);
    out.current_point = ToPoint2D(track.current_point);
    out.status = track.status;
    out.inlier = track.inlier;
    out.error = track.error;
    return out;
}

MaskPropagationData ToMaskPropagationData(const ORB_SLAM3::MaskPropagationTelemetry& telemetry)
{
    MaskPropagationData out;
    out.attempted = telemetry.attempted;
    out.succeeded = telemetry.succeeded;
    out.seed_points.reserve(telemetry.seed_points.size());
    for(const ORB_SLAM3::TelemetryPoint2D& point : telemetry.seed_points)
        out.seed_points.push_back(ToPoint2D(point));
    out.flowed_points.reserve(telemetry.flowed_points.size());
    for(const ORB_SLAM3::TelemetryPoint2D& point : telemetry.flowed_points)
        out.flowed_points.push_back(ToPoint2D(point));
    out.tracked_points.reserve(telemetry.tracked_points.size());
    for(const ORB_SLAM3::TelemetryPoint3D& point : telemetry.tracked_points)
        out.tracked_points.push_back(ToPoint3D(point));
    out.clusters.reserve(telemetry.clusters.size());
    for(const ORB_SLAM3::TelemetryCluster& cluster : telemetry.clusters)
    {
        PointCluster mapped_cluster;
        mapped_cluster.id = cluster.id;
        mapped_cluster.points.reserve(cluster.points.size());
        for(const ORB_SLAM3::TelemetryPoint3D& point : cluster.points)
            mapped_cluster.points.push_back(ToPoint3D(point));
        out.clusters.push_back(std::move(mapped_cluster));
    }
    return out;
}

FrameData ToFrameData(const ORB_SLAM3::FrameTelemetry& telemetry)
{
    FrameData out;
    out.frame_id = telemetry.frame_id;
    out.timestamp = telemetry.timestamp;
    out.depth_source_frame_id = telemetry.depth_source_frame_id;
    out.yolo_source_frame_id = telemetry.yolo_source_frame_id;
    out.mask_source_frame_id = telemetry.mask_source_frame_id;
    out.has_pose = telemetry.has_pose;
    out.has_dynamic_mask = !telemetry.dynamic_mask.empty();
    out.has_estimated_depth = !telemetry.estimated_depth.empty();
    out.used_optical_flow = telemetry.used_optical_flow;
    out.requested_new_keyframe = telemetry.requested_new_keyframe;
    out.is_keyframe = telemetry.is_keyframe;
    out.yolo_output_consumed = telemetry.yolo_output_consumed;
    out.mask_updated_from_yolo = telemetry.mask_updated_from_yolo;
    out.mask_updated_from_propagation = telemetry.mask_updated_from_propagation;
    out.has_reference_keyframe = telemetry.has_reference_keyframe;
    out.reference_keyframe_id = telemetry.reference_keyframe_id;
    out.tracking_ok = telemetry.tracking_ok;
    out.tracking_state = telemetry.tracking_state;
    out.tracking_status = ToTrackingStatus(out.tracking_state);
    out.tracking_state_name = TrackingStateName(out.tracking_state);
    out.tracked_map_point_count = telemetry.tracked_map_point_count;
    out.active_map_keyframe_count = telemetry.active_map_keyframe_count;
    out.active_map_point_count = telemetry.active_map_point_count;
    if(telemetry.has_pose)
        out.pose_matrix = ToMatrixArray(telemetry.pose);
    out.dynamic_mask = telemetry.dynamic_mask.clone();
    out.estimated_depth = telemetry.estimated_depth.clone();
    out.raw_orb_keypoints.reserve(telemetry.raw_orb_keypoints.size());
    for(const cv::KeyPoint& keypoint : telemetry.raw_orb_keypoints)
        out.raw_orb_keypoints.push_back(ToKeyPoint(keypoint));
    out.filtered_orb_keypoints.reserve(telemetry.filtered_orb_keypoints.size());
    for(const cv::KeyPoint& keypoint : telemetry.filtered_orb_keypoints)
        out.filtered_orb_keypoints.push_back(ToKeyPoint(keypoint));
    out.orb_keypoints.reserve(telemetry.orb_keypoints.size());
    for(const cv::KeyPoint& keypoint : telemetry.orb_keypoints)
        out.orb_keypoints.push_back(ToKeyPoint(keypoint));
    out.tracked_keypoints.reserve(telemetry.tracked_keypoints.size());
    for(const cv::KeyPoint& keypoint : telemetry.tracked_keypoints)
        out.tracked_keypoints.push_back(ToKeyPoint(keypoint));
    out.tracked_keypoint_count = out.tracked_keypoints.size();
    out.tracked_map_points_world.reserve(telemetry.tracked_map_points_world.size());
    for(const ORB_SLAM3::TelemetryPoint3D& point : telemetry.tracked_map_points_world)
        out.tracked_map_points_world.push_back(ToPoint3D(point));
    out.yolo_detections.reserve(telemetry.yolo_detections.size());
    for(const ORB_SLAM3::TelemetryDetection& detection : telemetry.yolo_detections)
        out.yolo_detections.push_back(ToDetectionBox(detection));
    out.optical_flow_tracks.reserve(telemetry.optical_flow_tracks.size());
    for(const ORB_SLAM3::TelemetryOpticalFlowTrack& track : telemetry.optical_flow_tracks)
        out.optical_flow_tracks.push_back(ToOpticalFlowTrack(track));
    out.mask_propagation = ToMaskPropagationData(telemetry.mask_propagation);
    return out;
}

} // namespace

struct MonocularPipeline::Impl
{
    std::mutex mutex;
    SessionConfig config;
    std::unique_ptr<ORB_SLAM3::System> system;
    FrameData current_frame_data;
    FrameData previous_frame_data;
};

MonocularPipeline::MonocularPipeline()
    : mImpl(new Impl())
{
}

MonocularPipeline::~MonocularPipeline()
{
    Shutdown();
}

bool MonocularPipeline::Initialize(const SessionConfig& config)
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(mImpl->system)
        return true;

    ORB_SLAM3::System::eSensor sensor = ORB_SLAM3::System::MONOCULAR;
    if(config.sensor_mode == SensorMode::ImuMonocular)
        sensor = ORB_SLAM3::System::IMU_MONOCULAR;

    mImpl->system.reset(new ORB_SLAM3::System(
        config.vocabulary_path,
        config.settings_path,
        sensor,
        config.use_viewer,
        config.init_frame,
        config.sequence_name));
    mImpl->config = config;
    mImpl->current_frame_data = FrameData();
    mImpl->previous_frame_data = FrameData();
    return true;
}

void MonocularPipeline::Shutdown()
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->system)
        return;

    mImpl->system->Shutdown();
    mImpl->system.reset();
}

bool MonocularPipeline::IsInitialized() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return static_cast<bool>(mImpl->system);
}

FrameData MonocularPipeline::ProcessFrame(const cv::Mat& rgb, double timestamp, const std::string& frame_name)
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->system)
        throw std::runtime_error("MonocularPipeline is not initialized.");
    if(rgb.empty())
        throw std::runtime_error("MonocularPipeline received an empty frame.");

    FrameData out;
    out.timestamp = timestamp;

    const Sophus::SE3f pose = mImpl->system->TrackMonocular(rgb, timestamp, std::vector<ORB_SLAM3::IMU::Point>(), frame_name);
    (void)pose;
    mImpl->previous_frame_data = ToFrameData(mImpl->system->GetPreviousFrameTelemetry());
    out = ToFrameData(mImpl->system->GetCurrentFrameTelemetry());
    out.frame_bgr = ToBgrFrame(rgb);
    mImpl->current_frame_data = out;
    return out;
}

FrameData MonocularPipeline::GetCurrentFrameData() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->current_frame_data;
}

FrameData MonocularPipeline::GetPreviousFrameData() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->previous_frame_data;
}

FrameData MonocularPipeline::GetLastFrameData() const
{
    return GetCurrentFrameData();
}

void MonocularPipeline::SaveTrajectoryEuRoC(const std::string& path) const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->system)
        throw std::runtime_error("MonocularPipeline is not initialized.");
    mImpl->system->SaveTrajectoryEuRoC(path);
}

void MonocularPipeline::SaveKeyFrameTrajectoryTUM(const std::string& path) const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->system)
        throw std::runtime_error("MonocularPipeline is not initialized.");
    mImpl->system->SaveKeyFrameTrajectoryTUM(path);
}

} // namespace monocular_slam
