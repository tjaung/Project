#ifndef DEPTH_ANYTHING_V2_H
#define DEPTH_ANYTHING_V2_H

#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

class DepthAnythingV2
{
public:
    DepthAnythingV2(const std::string& modelPath, int targetMinSide = 256, int intraOpThreads = 1);
    ~DepthAnythingV2();

    bool IsReady() const;
    const std::string& GetModelPath() const;
    int GetTargetMinSide() const;

    bool Infer(const cv::Mat& bgrImage, cv::Mat& depth8u);
    void Submit(int frameId, const cv::Mat& bgrImage);
    bool WaitFor(int frameId, cv::Mat& depth8u, int timeoutMs = -1);
    bool GetLatestCompleted(int& frameId, cv::Mat& depth8u);
    void ResetQueue();

private:
    class Impl;

    float ComputeScaleFactor(const cv::Size& size) const;
    bool PrepareInput(const cv::Mat& bgrImage, float scaleFactor);
    void WorkerLoop();

    std::string mModelPath;

    std::vector<float> mvInputData;
    std::array<int64_t, 4> mInputShape;
    int mInputHeight;
    int mInputWidth;
    int mTargetMinSide;
    bool mbReady;

    std::thread mWorkerThread;
    std::mutex mMutex;
    std::condition_variable mCondition;
    cv::Mat mPendingImage;
    int mPendingFrameId;
    bool mbHasPendingRequest;
    cv::Mat mCompletedDepth;
    int mCompletedFrameId;
    bool mbHasCompletedResult;
    bool mbStopWorker;

    std::unique_ptr<Impl> mpImpl;
};

} // namespace ORB_SLAM3

#endif
