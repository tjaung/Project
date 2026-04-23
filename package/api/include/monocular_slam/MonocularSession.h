#ifndef MONOCULAR_SLAM_MONOCULAR_SESSION_H
#define MONOCULAR_SLAM_MONOCULAR_SESSION_H

#include <memory>
#include <string>

#include "Config.h"
#include "Types.h"

namespace monocular_slam
{

class MonocularSession
{
public:
    class ARApi
    {
    public:
        ARApi();

        Anchor CreateAnchorAtCurrentPose(const std::string& label = "");
        Anchor CreateAnchor(const std::array<float, 16>& world_pose, const std::string& label = "");
        std::vector<Anchor> GetAnchors() const;
        bool RemoveAnchor(std::uint64_t anchor_id);
        void ClearAnchors();

    private:
        explicit ARApi(MonocularSession* owner);
        MonocularSession* mOwner;

        friend class MonocularSession;
    };

    MonocularSession();
    ~MonocularSession();

    bool Initialize(const SessionConfig& config);
    void Shutdown();

    FrameData ProcessFrame(const cv::Mat& rgb, double timestamp, const std::string& frame_name = std::string());

    bool StartVideo(const VideoSourceConfig& config);
    bool StartDatasetRun(const DatasetRunConfig& config);
    bool OpenVideo(const VideoSourceConfig& config);
    bool StepVideoFrame();
    void FinalizeVideo();
    bool OpenDatasetRun(const DatasetRunConfig& config);
    bool StepDatasetFrame();
    void FinalizeDatasetRun();

    void Stop();
    void Pause();
    void Resume();
    void Wait();

    bool IsInitialized() const;
    bool IsRunning() const;
    bool IsPaused() const;
    RunnerState GetRunnerState() const;
    SessionSnapshot GetSnapshot() const;

    FrameData GetCurrentFrameData() const;
    FrameData GetPreviousFrameData() const;
    RunMetrics GetRunMetrics() const;

    ARApi ar;

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace monocular_slam

#endif
