#ifndef MONOCULAR_SLAM_CONFIG_H
#define MONOCULAR_SLAM_CONFIG_H

#include <string>

namespace monocular_slam
{

enum class SensorMode
{
    Monocular = 0,
    ImuMonocular = 3
};

struct SessionConfig
{
    std::string vocabulary_path;
    std::string settings_path;
    bool use_viewer = false;
    int init_frame = 0;
    std::string sequence_name;
    SensorMode sensor_mode = SensorMode::Monocular;
};

struct VideoSourceConfig
{
    std::string source;
    bool realtime_playback = true;
};

struct DatasetRunConfig
{
    std::string dataset_path;
    bool realtime_playback = true;
    bool save_trajectories = true;
    std::string trajectory_path = "CameraTrajectory.txt";
    std::string keyframe_trajectory_path = "KeyFrameTrajectory.txt";
    bool evaluate_metrics = true;
    std::string metrics_csv_path = "out/monocular_session_metrics.csv";
};

} // namespace monocular_slam

#endif
