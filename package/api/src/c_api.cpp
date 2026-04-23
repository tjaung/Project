#include "monocular_slam/c_api.h"

#include <array>
#include <exception>
#include <string>

#include <opencv2/core/core.hpp>

#include "monocular_slam/MonocularSession.h"

struct mdslam_handle
{
    monocular_slam::MonocularSession session;
    std::string last_error;
    mdslam_error_code last_error_code = MDSLAM_SUCCESS;
};

namespace
{

std::string SafeString(const char* value)
{
    return value ? std::string(value) : std::string();
}

void ClearError(mdslam_handle* handle)
{
    if(handle)
    {
        handle->last_error.clear();
        handle->last_error_code = MDSLAM_SUCCESS;
    }
}

void SetError(mdslam_handle* handle, mdslam_error_code code, const std::string& message)
{
    if(handle)
    {
        handle->last_error = message;
        handle->last_error_code = code;
    }
}

monocular_slam::SensorMode ToSensorMode(int value)
{
    return value == MDSLAM_SENSOR_IMU_MONOCULAR ? monocular_slam::SensorMode::ImuMonocular
                                                : monocular_slam::SensorMode::Monocular;
}

monocular_slam::SessionConfig ToCppConfig(const mdslam_session_config* config)
{
    monocular_slam::SessionConfig out;
    if(!config)
        return out;
    out.vocabulary_path = SafeString(config->vocabulary_path);
    out.settings_path = SafeString(config->settings_path);
    out.use_viewer = (config->use_viewer != 0);
    out.init_frame = config->init_frame;
    out.sequence_name = SafeString(config->sequence_name);
    out.sensor_mode = ToSensorMode(config->sensor_mode);
    return out;
}

monocular_slam::VideoSourceConfig ToCppVideoConfig(const mdslam_video_config* config)
{
    monocular_slam::VideoSourceConfig out;
    if(!config)
        return out;
    out.source = SafeString(config->source);
    out.realtime_playback = (config->realtime_playback != 0);
    return out;
}

monocular_slam::DatasetRunConfig ToCppDatasetConfig(const mdslam_dataset_config* config)
{
    monocular_slam::DatasetRunConfig out;
    if(!config)
        return out;
    out.dataset_path = SafeString(config->dataset_path);
    out.realtime_playback = (config->realtime_playback != 0);
    out.save_trajectories = (config->save_trajectories != 0);
    if(config->trajectory_path)
        out.trajectory_path = config->trajectory_path;
    if(config->keyframe_trajectory_path)
        out.keyframe_trajectory_path = config->keyframe_trajectory_path;
    out.evaluate_metrics = (config->evaluate_metrics != 0);
    if(config->metrics_csv_path)
        out.metrics_csv_path = config->metrics_csv_path;
    return out;
}

int ToCRunnerState(monocular_slam::RunnerState state)
{
    switch(state)
    {
        case monocular_slam::RunnerState::Running: return MDSLAM_RUNNER_RUNNING;
        case monocular_slam::RunnerState::Paused: return MDSLAM_RUNNER_PAUSED;
        case monocular_slam::RunnerState::Finished: return MDSLAM_RUNNER_FINISHED;
        case monocular_slam::RunnerState::Failed: return MDSLAM_RUNNER_FAILED;
        case monocular_slam::RunnerState::Idle:
        default:
            return MDSLAM_RUNNER_IDLE;
    }
}

void CopyPose(const std::array<float, 16>& pose, float out_pose[16])
{
    for(size_t i = 0; i < 16; ++i)
        out_pose[i] = pose[i];
}

void FillSnapshot(const monocular_slam::SessionSnapshot& src, mdslam_snapshot* dst)
{
    if(!dst)
        return;
    dst->initialized = src.initialized ? 1 : 0;
    dst->running = src.running ? 1 : 0;
    dst->paused = src.paused ? 1 : 0;
    dst->runner_state = ToCRunnerState(src.runner_state);
    dst->frame_id = src.current_frame.frame_id;
    dst->timestamp = src.current_frame.timestamp;
    dst->tracking_state = src.current_frame.tracking_state;
    dst->has_pose = src.current_frame.has_pose ? 1 : 0;
    dst->has_dynamic_mask = src.current_frame.has_dynamic_mask ? 1 : 0;
    dst->has_estimated_depth = src.current_frame.has_estimated_depth ? 1 : 0;
    dst->tracking_ok = src.current_frame.tracking_ok ? 1 : 0;
    dst->requested_new_keyframe = src.current_frame.requested_new_keyframe ? 1 : 0;
    dst->is_keyframe = src.current_frame.is_keyframe ? 1 : 0;
    dst->tracked_keypoint_count = src.current_frame.tracked_keypoint_count;
    dst->tracked_map_point_count = src.current_frame.tracked_map_point_count;
    dst->active_map_keyframe_count = src.current_frame.active_map_keyframe_count;
    dst->active_map_point_count = src.current_frame.active_map_point_count;
    CopyPose(src.current_frame.pose_matrix, dst->pose_matrix);
}

mdslam_run_metrics FillRunMetrics(const monocular_slam::RunMetrics& src)
{
    mdslam_run_metrics out = {};
    out.frames_processed = src.frames_processed;
    out.mean_tracking_time_sec = src.mean_tracking_time_sec;
    out.median_tracking_time_sec = src.median_tracking_time_sec;
    out.trajectories_saved = src.trajectories_saved ? 1 : 0;
    out.metric_evaluation_attempted = src.metric_evaluation_attempted ? 1 : 0;
    out.metric_evaluation_succeeded = src.metric_evaluation_succeeded ? 1 : 0;
    out.metric_evaluator_exit_code = src.metric_evaluator_exit_code;
    return out;
}

void FillAnchor(const monocular_slam::Anchor& src, mdslam_anchor* dst)
{
    if(!dst)
        return;
    dst->id = src.id;
    dst->valid = src.valid ? 1 : 0;
    dst->source_frame_id = src.source_frame_id;
    dst->timestamp = src.timestamp;
    CopyPose(src.pose_matrix, dst->pose_matrix);
}

} // namespace

extern "C" {

mdslam_handle* mdslam_create(void)
{
    try
    {
        return new mdslam_handle();
    }
    catch(...)
    {
        return nullptr;
    }
}

void mdslam_destroy(mdslam_handle* handle)
{
    delete handle;
}

int mdslam_initialize(mdslam_handle* handle, const mdslam_session_config* config)
{
    if(!handle)
        return 0;
    if(!config)
    {
        SetError(handle, MDSLAM_ERROR_INVALID_ARGUMENT, "Null config passed to mdslam_initialize.");
        return 0;
    }

    try
    {
        ClearError(handle);
        return handle->session.Initialize(ToCppConfig(config)) ? 1 : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

void mdslam_shutdown(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.Shutdown();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

int mdslam_is_initialized(const mdslam_handle* handle)
{
    return (handle && handle->session.IsInitialized()) ? 1 : 0;
}

int mdslam_process_frame_bgr8(mdslam_handle* handle, const mdslam_frame* frame)
{
    if(!handle)
        return 0;
    if(!frame || !frame->bgr_data || frame->width <= 0 || frame->height <= 0 || frame->stride_bytes <= 0)
    {
        SetError(handle, MDSLAM_ERROR_INVALID_ARGUMENT, "Invalid frame buffer or dimensions.");
        return 0;
    }

    try
    {
        ClearError(handle);
        cv::Mat bgr(frame->height,
                    frame->width,
                    CV_8UC3,
                    const_cast<unsigned char*>(frame->bgr_data),
                    static_cast<size_t>(frame->stride_bytes));
        handle->session.ProcessFrame(bgr.clone(), frame->timestamp, SafeString(frame->frame_name));
        return 1;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

int mdslam_open_video(mdslam_handle* handle, const mdslam_video_config* config)
{
    if(!handle)
        return 0;
    if(!config)
    {
        SetError(handle, MDSLAM_ERROR_INVALID_ARGUMENT, "Null config passed to mdslam_open_video.");
        return 0;
    }

    try
    {
        ClearError(handle);
        return handle->session.OpenVideo(ToCppVideoConfig(config)) ? 1 : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

int mdslam_step_video(mdslam_handle* handle)
{
    if(!handle)
        return 0;
    try
    {
        ClearError(handle);
        return handle->session.StepVideoFrame() ? 1 : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

void mdslam_finalize_video(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.FinalizeVideo();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

int mdslam_open_dataset(mdslam_handle* handle, const mdslam_dataset_config* config)
{
    if(!handle)
        return 0;
    if(!config)
    {
        SetError(handle, MDSLAM_ERROR_INVALID_ARGUMENT, "Null config passed to mdslam_open_dataset.");
        return 0;
    }

    try
    {
        ClearError(handle);
        return handle->session.OpenDatasetRun(ToCppDatasetConfig(config)) ? 1 : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

int mdslam_step_dataset(mdslam_handle* handle)
{
    if(!handle)
        return 0;
    try
    {
        ClearError(handle);
        return handle->session.StepDatasetFrame() ? 1 : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

void mdslam_finalize_dataset(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.FinalizeDatasetRun();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

void mdslam_stop(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.Stop();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

void mdslam_pause(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.Pause();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

void mdslam_resume(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.Resume();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

void mdslam_wait(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.Wait();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

int mdslam_get_snapshot(const mdslam_handle* handle, mdslam_snapshot* out_snapshot)
{
    if(!handle || !out_snapshot)
        return 0;
    try
    {
        FillSnapshot(handle->session.GetSnapshot(), out_snapshot);
        return 1;
    }
    catch(...)
    {
        return 0;
    }
}

mdslam_run_metrics mdslam_get_run_metrics(const mdslam_handle* handle)
{
    if(!handle)
    {
        mdslam_run_metrics empty = {};
        return empty;
    }
    return FillRunMetrics(handle->session.GetRunMetrics());
}

uint64_t mdslam_create_anchor_at_current_pose(mdslam_handle* handle, const char* label)
{
    if(!handle)
        return 0;
    try
    {
        ClearError(handle);
        const monocular_slam::Anchor anchor = handle->session.ar.CreateAnchorAtCurrentPose(SafeString(label));
        return anchor.valid ? anchor.id : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

uint64_t mdslam_create_anchor(mdslam_handle* handle, const float pose_matrix[16], const char* label)
{
    if(!handle || !pose_matrix)
        return 0;
    try
    {
        ClearError(handle);
        std::array<float, 16> pose{};
        for(size_t i = 0; i < 16; ++i)
            pose[i] = pose_matrix[i];
        const monocular_slam::Anchor anchor = handle->session.ar.CreateAnchor(pose, SafeString(label));
        return anchor.valid ? anchor.id : 0;
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
        return 0;
    }
}

size_t mdslam_get_anchor_count(const mdslam_handle* handle)
{
    if(!handle)
        return 0;
    return handle->session.ar.GetAnchors().size();
}

int mdslam_get_anchor(const mdslam_handle* handle, size_t index, mdslam_anchor* out_anchor)
{
    if(!handle || !out_anchor)
        return 0;
    const std::vector<monocular_slam::Anchor> anchors = handle->session.ar.GetAnchors();
    if(index >= anchors.size())
        return 0;
    FillAnchor(anchors[index], out_anchor);
    return 1;
}

void mdslam_clear_anchors(mdslam_handle* handle)
{
    if(!handle)
        return;
    try
    {
        ClearError(handle);
        handle->session.ar.ClearAnchors();
    }
    catch(const std::exception& e)
    {
        SetError(handle, MDSLAM_ERROR_EXCEPTION, e.what());
    }
}

const char* mdslam_get_last_error(const mdslam_handle* handle)
{
    if(!handle)
        return "";
    return handle->last_error.c_str();
}

mdslam_error_code mdslam_get_last_error_code(const mdslam_handle* handle)
{
    if(!handle)
        return MDSLAM_ERROR_NULL_HANDLE;
    return handle->last_error_code;
}

} // extern "C"
