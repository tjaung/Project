#include "DepthAnythingV2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include <opencv2/imgproc.hpp>

#ifdef ORB_SLAM3_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace ORB_SLAM3
{

#ifdef ORB_SLAM3_HAS_ONNXRUNTIME
class DepthAnythingV2::Impl
{
public:
    Impl()
        : env(ORT_LOGGING_LEVEL_WARNING, "depth-anything-v2")
    {
    }

    Ort::Env env;
    Ort::SessionOptions sessionOptions;
    std::unique_ptr<Ort::Session> session;
    std::string inputNameStorage;
    std::string outputNameStorage;
    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
};
#else
class DepthAnythingV2::Impl
{
};
#endif

DepthAnythingV2::DepthAnythingV2(const std::string& modelPath, int targetMinSide, int intraOpThreads)
    : mModelPath(modelPath),
      mInputShape{{1, 3, 0, 0}},
      mInputHeight(0),
      mInputWidth(0),
      mTargetMinSide(std::max(64, targetMinSide)),
      mbReady(false),
      mPendingFrameId(-1),
      mbHasPendingRequest(false),
      mCompletedFrameId(-1),
      mbHasCompletedResult(false),
      mbStopWorker(false),
      mpImpl(new Impl())
{
#ifdef ORB_SLAM3_HAS_ONNXRUNTIME
    mpImpl->inputNameStorage = "pixel_values";
    mpImpl->outputNameStorage = "predicted_depth";
    mpImpl->inputNames.push_back(mpImpl->inputNameStorage.c_str());
    mpImpl->outputNames.push_back(mpImpl->outputNameStorage.c_str());

    try
    {
        mpImpl->sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        mpImpl->sessionOptions.SetIntraOpNumThreads(std::max(1, intraOpThreads));
        mpImpl->session.reset(new Ort::Session(mpImpl->env, mModelPath.c_str(), mpImpl->sessionOptions));
        mbReady = true;
        mWorkerThread = std::thread(&DepthAnythingV2::WorkerLoop, this);
    }
    catch(const Ort::Exception& e)
    {
        std::cerr << "Failed to initialize Depth Anything V2 session: " << e.what() << std::endl;
        mpImpl->session.reset();
        mbReady = false;
    }
#else
    (void)intraOpThreads;
    std::cerr << "Depth Anything V2 support was requested, but this build has no ONNX Runtime." << std::endl;
#endif
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
#ifdef ORB_SLAM3_HAS_ONNXRUNTIME
    return mbReady && mpImpl && mpImpl->session.get() != nullptr;
#else
    return false;
#endif
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
#ifndef ORB_SLAM3_HAS_ONNXRUNTIME
    (void)bgrImage;
    depth8u.release();
    return false;
#else
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
        std::vector<Ort::Value> outputTensors = mpImpl->session->Run(
            runOptions,
            mpImpl->inputNames.data(),
            &inputTensor,
            1,
            mpImpl->outputNames.data(),
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
#endif
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
#ifndef ORB_SLAM3_HAS_ONNXRUNTIME
    (void)bgrImage;
    (void)scaleFactor;
    return false;
#else
    cv::Mat scaledImage;
    if(std::fabs(scaleFactor - 1.0f) > 1e-5f)
    {
        cv::resize(bgrImage, scaledImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);
    }
    else
    {
        scaledImage = bgrImage;
    }

    cv::Mat rgbImage;
    cv::cvtColor(scaledImage, rgbImage, cv::COLOR_BGR2RGB);

    rgbImage.convertTo(rgbImage, CV_32FC3, 1.0 / 255.0);

    mInputHeight = rgbImage.rows;
    mInputWidth = rgbImage.cols;
    if(mInputHeight <= 0 || mInputWidth <= 0)
        return false;

    mInputShape = {{1, 3, mInputHeight, mInputWidth}};
    mvInputData.resize(static_cast<size_t>(3 * mInputHeight * mInputWidth));

    for(int y = 0; y < mInputHeight; ++y)
    {
        const cv::Vec3f* row = rgbImage.ptr<cv::Vec3f>(y);
        for(int x = 0; x < mInputWidth; ++x)
        {
            const cv::Vec3f& pixel = row[x];
            const int idx = y * mInputWidth + x;
            mvInputData[idx] = pixel[0];
            mvInputData[mInputHeight * mInputWidth + idx] = pixel[1];
            mvInputData[2 * mInputHeight * mInputWidth + idx] = pixel[2];
        }
    }

    return true;
#endif
}

void DepthAnythingV2::WorkerLoop()
{
    while(true)
    {
        cv::Mat requestImage;
        int requestFrameId = -1;

        {
            std::unique_lock<std::mutex> lock(mMutex);
            mCondition.wait(lock, [this]()
            {
                return mbStopWorker || mbHasPendingRequest;
            });

            if(mbStopWorker)
                break;

            requestFrameId = mPendingFrameId;
            requestImage = mPendingImage.clone();
            mbHasPendingRequest = false;
        }

        cv::Mat depth;
        if(Infer(requestImage, depth))
        {
            {
                std::lock_guard<std::mutex> lock(mMutex);
                mCompletedFrameId = requestFrameId;
                mCompletedDepth = depth;
                mbHasCompletedResult = !mCompletedDepth.empty();
            }
            mCondition.notify_all();
        }
    }
}

} // namespace ORB_SLAM3
