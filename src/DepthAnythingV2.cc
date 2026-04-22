#include "DepthAnythingV2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3
{

DepthAnythingV2::DepthAnythingV2(const std::string& modelPath, int targetMinSide, int intraOpThreads)
    : mEnv(ORT_LOGGING_LEVEL_WARNING, "depth-anything-v2"),
      mModelPath(modelPath),
      mInputNameStorage("pixel_values"),
      mOutputNameStorage("predicted_depth"),
      mInputShape{{1, 3, 0, 0}},
      mInputHeight(0),
      mInputWidth(0),
      mTargetMinSide(std::max(64, targetMinSide)),
      mbReady(false),
      mPendingFrameId(-1),
      mbHasPendingRequest(false),
      mCompletedFrameId(-1),
      mbHasCompletedResult(false),
      mbStopWorker(false)
{
    mvInputNames.push_back(mInputNameStorage.c_str());
    mvOutputNames.push_back(mOutputNameStorage.c_str());

    try
    {
        mSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        mSessionOptions.SetIntraOpNumThreads(std::max(1, intraOpThreads));
        mpSession.reset(new Ort::Session(mEnv, mModelPath.c_str(), mSessionOptions));
        mbReady = true;
        mWorkerThread = std::thread(&DepthAnythingV2::WorkerLoop, this);
    }
    catch(const Ort::Exception& e)
    {
        std::cerr << "Failed to initialize Depth Anything V2 session: " << e.what() << std::endl;
        mpSession.reset();
        mbReady = false;
    }
}

DepthAnythingV2::~DepthAnythingV2()
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mbStopWorker = true;
    }
    mCondition.notify_all();

    if(mWorkerThread.joinable())
        mWorkerThread.join();
}

bool DepthAnythingV2::IsReady() const
{
    return mbReady && mpSession.get() != nullptr;
}

const std::string& DepthAnythingV2::GetModelPath() const
{
    return mModelPath;
}

int DepthAnythingV2::GetTargetMinSide() const
{
    return mTargetMinSide;
}

bool DepthAnythingV2::Infer(const cv::Mat& bgrImage, cv::Mat& depth8u)
{
    if(!IsReady() || bgrImage.empty())
        return false;

    cv::Mat bgr;
    if(bgrImage.channels() == 3)
    {
        bgr = bgrImage;
    }
    else if(bgrImage.channels() == 1)
    {
        cv::cvtColor(bgrImage, bgr, cv::COLOR_GRAY2BGR);
    }
    else if(bgrImage.channels() == 4)
    {
        cv::cvtColor(bgrImage, bgr, cv::COLOR_BGRA2BGR);
    }
    else
    {
        return false;
    }

    const float scaleFactor = ComputeScaleFactor(bgr.size());
    if(!PrepareInput(bgr, scaleFactor))
        return false;

    try
    {
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo,
            mvInputData.data(),
            mvInputData.size(),
            mInputShape.data(),
            mInputShape.size());

        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputTensors = mpSession->Run(
            runOptions,
            mvInputNames.data(),
            &inputTensor,
            1,
            mvOutputNames.data(),
            1);

        if(outputTensors.empty() || !outputTensors[0].IsTensor())
            return false;

        Ort::TensorTypeAndShapeInfo outputInfo = outputTensors[0].GetTensorTypeAndShapeInfo();
        std::vector<int64_t> outputShape = outputInfo.GetShape();
        if(outputShape.size() < 2)
            return false;

        const int outHeight = static_cast<int>(outputShape[outputShape.size() - 2]);
        const int outWidth = static_cast<int>(outputShape[outputShape.size() - 1]);
        if(outHeight <= 0 || outWidth <= 0)
            return false;

        const float* tensorData = outputTensors[0].GetTensorData<float>();
        const int outputSize = outHeight * outWidth;

        float minValue = tensorData[0];
        float maxValue = tensorData[0];
        for(int i = 1; i < outputSize; ++i)
        {
            minValue = std::min(minValue, tensorData[i]);
            maxValue = std::max(maxValue, tensorData[i]);
        }

        cv::Mat rawDepth(outHeight, outWidth, CV_8UC1);
        if(std::fabs(maxValue - minValue) < 1e-6f)
        {
            rawDepth.setTo(cv::Scalar(0));
        }
        else
        {
            const float scale = 255.0f / (maxValue - minValue);
            for(int y = 0, k = 0; y < outHeight; ++y)
            {
                unsigned char* ptr = rawDepth.ptr<unsigned char>(y);
                for(int x = 0; x < outWidth; ++x, ++k)
                {
                    const float value = (tensorData[k] - minValue) * scale;
                    ptr[x] = static_cast<unsigned char>(std::max(0.0f, std::min(255.0f, value)));
                }
            }
        }

        cv::resize(rawDepth, depth8u, bgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
        return true;
    }
    catch(const Ort::Exception& e)
    {
        std::cerr << "Depth Anything V2 inference failed: " << e.what() << std::endl;
        return false;
    }
}

void DepthAnythingV2::Submit(int frameId, const cv::Mat& bgrImage)
{
    if(!IsReady() || bgrImage.empty())
        return;

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mPendingFrameId = frameId;
        mPendingImage = bgrImage.clone();
        mbHasPendingRequest = true;
    }

    mCondition.notify_all();
}

bool DepthAnythingV2::WaitFor(int frameId, cv::Mat& depth8u, int timeoutMs)
{
    if(!IsReady())
        return false;

    std::unique_lock<std::mutex> lock(mMutex);
    auto readyPredicate = [this, frameId]()
    {
        return mbStopWorker || (mbHasCompletedResult && mCompletedFrameId == frameId);
    };

    bool ready = false;
    if(timeoutMs < 0)
    {
        mCondition.wait(lock, readyPredicate);
        ready = readyPredicate();
    }
    else
    {
        ready = mCondition.wait_for(lock, std::chrono::milliseconds(timeoutMs), readyPredicate);
    }

    if(!ready || mbStopWorker || !mbHasCompletedResult || mCompletedFrameId != frameId)
        return false;

    depth8u = mCompletedDepth.clone();
    return !depth8u.empty();
}

bool DepthAnythingV2::GetLatestCompleted(int& frameId, cv::Mat& depth8u)
{
    if(!IsReady())
        return false;

    std::lock_guard<std::mutex> lock(mMutex);
    if(!mbHasCompletedResult || mCompletedDepth.empty())
        return false;

    frameId = mCompletedFrameId;
    depth8u = mCompletedDepth.clone();
    return true;
}

void DepthAnythingV2::ResetQueue()
{
    std::lock_guard<std::mutex> lock(mMutex);
    mPendingImage.release();
    mPendingFrameId = -1;
    mbHasPendingRequest = false;
    mCompletedDepth.release();
    mCompletedFrameId = -1;
    mbHasCompletedResult = false;
}

float DepthAnythingV2::ComputeScaleFactor(const cv::Size& size) const
{
    const int minSide = std::min(size.width, size.height);
    if(minSide <= 0 || minSide <= mTargetMinSide)
        return 1.0f;

    return static_cast<float>(mTargetMinSide) / static_cast<float>(minSide);
}

bool DepthAnythingV2::PrepareInput(const cv::Mat& bgrImage, float scaleFactor)
{
    cv::Mat scaledImage;
    if(std::fabs(scaleFactor - 1.0f) > 1e-5f)
    {
        cv::resize(bgrImage, scaledImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);
    }
    else
    {
        scaledImage = bgrImage;
    }

    if(scaledImage.empty())
        return false;

    if(scaledImage.rows != mInputHeight || scaledImage.cols != mInputWidth)
    {
        mInputHeight = scaledImage.rows;
        mInputWidth = scaledImage.cols;
        mInputShape[2] = mInputHeight;
        mInputShape[3] = mInputWidth;
        mvInputData.resize(static_cast<size_t>(mInputHeight) * static_cast<size_t>(mInputWidth) * 3u);
    }

    const int imageSize = mInputHeight * mInputWidth;
    for(int y = 0; y < scaledImage.rows; ++y)
    {
        const cv::Vec3b* srcPtr = scaledImage.ptr<cv::Vec3b>(y);
        float* dstR = &mvInputData[y * mInputWidth];
        float* dstG = &mvInputData[imageSize + y * mInputWidth];
        float* dstB = &mvInputData[imageSize * 2 + y * mInputWidth];

        for(int x = 0; x < scaledImage.cols; ++x)
        {
            dstR[x] = ((static_cast<float>(srcPtr[x][2]) / 255.0f) - 0.485f) / 0.229f;
            dstG[x] = ((static_cast<float>(srcPtr[x][1]) / 255.0f) - 0.456f) / 0.224f;
            dstB[x] = ((static_cast<float>(srcPtr[x][0]) / 255.0f) - 0.406f) / 0.225f;
        }
    }

    return true;
}

void DepthAnythingV2::WorkerLoop()
{
    while(true)
    {
        cv::Mat requestImage;
        int requestFrameId = -1;

        {
            std::unique_lock<std::mutex> lock(mMutex);
            mCondition.wait(lock, [this]() { return mbStopWorker || mbHasPendingRequest; });

            if(mbStopWorker)
                break;

            requestFrameId = mPendingFrameId;
            requestImage = mPendingImage.clone();
            mbHasPendingRequest = false;
        }

        cv::Mat depth8u;
        const bool ok = Infer(requestImage, depth8u);

        {
            std::lock_guard<std::mutex> lock(mMutex);
            if(ok)
            {
                mCompletedFrameId = requestFrameId;
                mCompletedDepth = depth8u;
                mbHasCompletedResult = true;
            }
        }
        mCondition.notify_all();
    }
}

} // namespace ORB_SLAM3
