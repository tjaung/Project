#ifndef MONOCULAR_SLAM_MONOCULAR_PIPELINE_H
#define MONOCULAR_SLAM_MONOCULAR_PIPELINE_H

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include "Config.h"
#include "Types.h"

namespace monocular_slam
{

class MonocularPipeline
{
public:
    MonocularPipeline();
    ~MonocularPipeline();

    bool Initialize(const SessionConfig& config);
    void Shutdown();

    bool IsInitialized() const;

    FrameData ProcessFrame(const cv::Mat& rgb, double timestamp, const std::string& frame_name = std::string());
    FrameData GetCurrentFrameData() const;
    FrameData GetPreviousFrameData() const;
    FrameData GetLastFrameData() const;
    void SaveTrajectoryEuRoC(const std::string& path) const;
    void SaveKeyFrameTrajectoryTUM(const std::string& path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace monocular_slam

#endif
