#ifndef MONOCULAR_SLAM_C_API_H
#define MONOCULAR_SLAM_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mdslam_handle mdslam_handle;

typedef enum mdslam_error_code
{
    MDSLAM_SUCCESS = 0,
    MDSLAM_ERROR_NULL_HANDLE = 1,
    MDSLAM_ERROR_INVALID_ARGUMENT = 2,
    MDSLAM_ERROR_NOT_INITIALIZED = 3,
    MDSLAM_ERROR_BUSY = 4,
    MDSLAM_ERROR_IO = 5,
    MDSLAM_ERROR_EXCEPTION = 6
} mdslam_error_code;

typedef enum mdslam_sensor_mode
{
    MDSLAM_SENSOR_MONOCULAR = 0,
    MDSLAM_SENSOR_IMU_MONOCULAR = 3
} mdslam_sensor_mode;

typedef enum mdslam_runner_state
{
    MDSLAM_RUNNER_IDLE = 0,
    MDSLAM_RUNNER_RUNNING = 1,
    MDSLAM_RUNNER_PAUSED = 2,
    MDSLAM_RUNNER_FINISHED = 3,
    MDSLAM_RUNNER_FAILED = 4
} mdslam_runner_state;

typedef struct mdslam_session_config
{
    const char* vocabulary_path;
    const char* settings_path;
    int use_viewer;
    int init_frame;
    const char* sequence_name;
    int sensor_mode;
} mdslam_session_config;

typedef struct mdslam_video_config
{
    const char* source;
    int realtime_playback;
} mdslam_video_config;

typedef struct mdslam_dataset_config
{
    const char* dataset_path;
    int realtime_playback;
    int save_trajectories;
    const char* trajectory_path;
    const char* keyframe_trajectory_path;
    int evaluate_metrics;
    const char* metrics_csv_path;
} mdslam_dataset_config;

typedef struct mdslam_frame
{
    const unsigned char* bgr_data;
    int width;
    int height;
    int stride_bytes;
    double timestamp;
    const char* frame_name;
} mdslam_frame;

typedef struct mdslam_snapshot
{
    int initialized;
    int running;
    int paused;
    int runner_state;
    uint64_t frame_id;
    double timestamp;
    int tracking_state;
    int has_pose;
    int has_dynamic_mask;
    int has_estimated_depth;
    int tracking_ok;
    int requested_new_keyframe;
    int is_keyframe;
    size_t tracked_keypoint_count;
    size_t tracked_map_point_count;
    size_t active_map_keyframe_count;
    size_t active_map_point_count;
    float pose_matrix[16];
} mdslam_snapshot;

typedef struct mdslam_run_metrics
{
    size_t frames_processed;
    double mean_tracking_time_sec;
    double median_tracking_time_sec;
    int trajectories_saved;
    int metric_evaluation_attempted;
    int metric_evaluation_succeeded;
    int metric_evaluator_exit_code;
} mdslam_run_metrics;

typedef struct mdslam_anchor
{
    uint64_t id;
    int valid;
    uint64_t source_frame_id;
    double timestamp;
    float pose_matrix[16];
} mdslam_anchor;

mdslam_handle* mdslam_create(void);
void mdslam_destroy(mdslam_handle* handle);

int mdslam_initialize(mdslam_handle* handle, const mdslam_session_config* config);
void mdslam_shutdown(mdslam_handle* handle);
int mdslam_is_initialized(const mdslam_handle* handle);

int mdslam_process_frame_bgr8(mdslam_handle* handle, const mdslam_frame* frame);

int mdslam_open_video(mdslam_handle* handle, const mdslam_video_config* config);
int mdslam_step_video(mdslam_handle* handle);
void mdslam_finalize_video(mdslam_handle* handle);

int mdslam_open_dataset(mdslam_handle* handle, const mdslam_dataset_config* config);
int mdslam_step_dataset(mdslam_handle* handle);
void mdslam_finalize_dataset(mdslam_handle* handle);

void mdslam_stop(mdslam_handle* handle);
void mdslam_pause(mdslam_handle* handle);
void mdslam_resume(mdslam_handle* handle);
void mdslam_wait(mdslam_handle* handle);

int mdslam_get_snapshot(const mdslam_handle* handle, mdslam_snapshot* out_snapshot);
mdslam_run_metrics mdslam_get_run_metrics(const mdslam_handle* handle);

uint64_t mdslam_create_anchor_at_current_pose(mdslam_handle* handle, const char* label);
uint64_t mdslam_create_anchor(mdslam_handle* handle, const float pose_matrix[16], const char* label);
size_t mdslam_get_anchor_count(const mdslam_handle* handle);
int mdslam_get_anchor(const mdslam_handle* handle, size_t index, mdslam_anchor* out_anchor);
void mdslam_clear_anchors(mdslam_handle* handle);

const char* mdslam_get_last_error(const mdslam_handle* handle);
mdslam_error_code mdslam_get_last_error_code(const mdslam_handle* handle);

#ifdef __cplusplus
}
#endif

#endif
