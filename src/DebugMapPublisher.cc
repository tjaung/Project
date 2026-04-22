#include "DebugMapPublisher.h"
#include "FrameDrawer.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <set>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3
{

DebugMapPublisher::DebugMapPublisher(Atlas* pAtlas, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const std::string& outputPath, int intervalMs)
    : mpAtlas(pAtlas),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mOutputPath(outputPath),
      mIntervalMs(intervalMs),
      mbFinishRequested(false),
      mbFinished(true)
{
}

void DebugMapPublisher::Run()
{
    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = false;
    }

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(mMutexFinish);
            if (mbFinishRequested)
            {
                mbFinished = true;
                break;
            }
        }

        WriteSnapshot();
        std::this_thread::sleep_for(std::chrono::milliseconds(mIntervalMs));
    }
}

void DebugMapPublisher::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool DebugMapPublisher::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

bool DebugMapPublisher::WriteFrameImage(const std::string& frameImagePath)
{
    if (!mpFrameDrawer)
        return false;

    cv::Mat frame = mpFrameDrawer->DrawFrame(1.0f);
    if (frame.empty())
        return false;

    if (mpFrameDrawer->both)
    {
        cv::Mat rightFrame = mpFrameDrawer->DrawRightFrame(1.0f);
        if (!rightFrame.empty())
        {
            cv::Mat combined;
            cv::hconcat(frame, rightFrame, combined);
            frame = combined;
        }
    }

    const std::string tempPath = frameImagePath + ".tmp.jpg";
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(90);
    if (!cv::imwrite(tempPath, frame, params))
        return false;

    return std::rename(tempPath.c_str(), frameImagePath.c_str()) == 0;
}

DebugMapPublisher::CameraSample DebugMapPublisher::SampleCamera() const
{
    CameraSample sample;
    Sophus::SE3f Twc = mpMapDrawer->GetCurrentCameraPose();
    sample.position = Twc.translation();
    sample.rotation = Twc.rotationMatrix();
    sample.forward = sample.rotation * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    return sample;
}

bool DebugMapPublisher::WriteSnapshot()
{
    if (!mpAtlas)
        return false;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return false;

    std::unique_lock<std::mutex> lock(pActiveMap->mMutexMapUpdate);

    const std::vector<MapPoint*> mapPoints = pActiveMap->GetAllMapPoints();
    const std::vector<MapPoint*> refPoints = pActiveMap->GetReferenceMapPoints();
    const std::vector<KeyFrame*> keyFrames = pActiveMap->GetAllKeyFrames();

    std::set<long unsigned int> refPointIds;
    for (MapPoint* pMP : refPoints)
    {
        if (pMP && !pMP->isBad())
            refPointIds.insert(pMP->mnId);
    }

    const CameraSample camera = SampleCamera();
    const std::string frameImagePath = mOutputPath + ".frame.jpg";
    WriteFrameImage(frameImagePath);
    const std::string tempPath = mOutputPath + ".tmp";
    std::ofstream out(tempPath.c_str(), std::ios::trunc);
    if (!out.is_open())
        return false;

    out.setf(std::ios::fixed);
    out.precision(6);

    out << "ORB_SLAM3_DEBUG_MAP_V1\n";
    out << "CAMERA "
        << camera.position.x() << " " << camera.position.y() << " " << camera.position.z() << " "
        << camera.rotation(0,0) << " " << camera.rotation(0,1) << " " << camera.rotation(0,2) << " "
        << camera.rotation(1,0) << " " << camera.rotation(1,1) << " " << camera.rotation(1,2) << " "
        << camera.rotation(2,0) << " " << camera.rotation(2,1) << " " << camera.rotation(2,2) << "\n";
    out << "FRAME " << frameImagePath << "\n";

    size_t validPointCount = 0;
    for (MapPoint* pMP : mapPoints)
    {
        if (pMP && !pMP->isBad())
            ++validPointCount;
    }

    out << "POINTS " << validPointCount << "\n";
    for (MapPoint* pMP : mapPoints)
    {
        if (!pMP || pMP->isBad())
            continue;
        const Eigen::Vector3f pos = pMP->GetWorldPos();
        const int isReference = refPointIds.count(pMP->mnId) ? 1 : 0;
        out << pos.x() << " " << pos.y() << " " << pos.z() << " " << isReference << "\n";
    }

    size_t validKeyframeCount = 0;
    for (KeyFrame* pKF : keyFrames)
    {
        if (pKF && !pKF->isBad())
            ++validKeyframeCount;
    }

    out << "KEYFRAMES " << validKeyframeCount << "\n";
    for (KeyFrame* pKF : keyFrames)
    {
        if (!pKF || pKF->isBad())
            continue;
        const Sophus::SE3f Twc = pKF->GetPoseInverse();
        const Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        const Eigen::Vector3f center = Twc.translation();
        const int isRoot = pKF->GetParent() ? 0 : 1;
        out << pKF->mnId << " "
            << center.x() << " " << center.y() << " " << center.z() << " "
            << Rwc(0,0) << " " << Rwc(0,1) << " " << Rwc(0,2) << " "
            << Rwc(1,0) << " " << Rwc(1,1) << " " << Rwc(1,2) << " "
            << Rwc(2,0) << " " << Rwc(2,1) << " " << Rwc(2,2) << " "
            << isRoot << " " << pKF->mnOriginMapId << "\n";
    }

    std::set<std::pair<long unsigned int, long unsigned int> > emittedCovisible;

    size_t edgeCount = 0;
    for (KeyFrame* pKF : keyFrames)
    {
        if (!pKF || pKF->isBad())
            continue;

        KeyFrame* pParent = pKF->GetParent();
        if (pParent && !pParent->isBad())
            ++edgeCount;

        const std::set<KeyFrame*> loopEdges = pKF->GetLoopEdges();
        for (KeyFrame* pLoopKF : loopEdges)
        {
            if (pLoopKF && !pLoopKF->isBad() && pLoopKF->mnId > pKF->mnId)
                ++edgeCount;
        }

        const std::vector<KeyFrame*> covisible = pKF->GetCovisiblesByWeight(100);
        for (KeyFrame* pCovKF : covisible)
        {
            if (!pCovKF || pCovKF->isBad())
                continue;
            const long unsigned int a = std::min(pKF->mnId, pCovKF->mnId);
            const long unsigned int b = std::max(pKF->mnId, pCovKF->mnId);
            if (a == b || emittedCovisible.count(std::make_pair(a, b)))
                continue;
            emittedCovisible.insert(std::make_pair(a, b));
            ++edgeCount;
        }

        if (pActiveMap->isImuInitialized() && pKF->mNextKF && !pKF->mNextKF->isBad())
            ++edgeCount;
    }

    out << "EDGES " << edgeCount << "\n";

    emittedCovisible.clear();
    for (KeyFrame* pKF : keyFrames)
    {
        if (!pKF || pKF->isBad())
            continue;

        const Eigen::Vector3f center = pKF->GetCameraCenter();

        KeyFrame* pParent = pKF->GetParent();
        if (pParent && !pParent->isBad())
        {
            const Eigen::Vector3f parentCenter = pParent->GetCameraCenter();
            out << center.x() << " " << center.y() << " " << center.z() << " "
                << parentCenter.x() << " " << parentCenter.y() << " " << parentCenter.z() << " "
                << 0 << "\n";
        }

        const std::vector<KeyFrame*> covisible = pKF->GetCovisiblesByWeight(100);
        for (KeyFrame* pCovKF : covisible)
        {
            if (!pCovKF || pCovKF->isBad())
                continue;

            const long unsigned int a = std::min(pKF->mnId, pCovKF->mnId);
            const long unsigned int b = std::max(pKF->mnId, pCovKF->mnId);
            if (a == b || emittedCovisible.count(std::make_pair(a, b)))
                continue;

            emittedCovisible.insert(std::make_pair(a, b));
            const Eigen::Vector3f covCenter = pCovKF->GetCameraCenter();
            out << center.x() << " " << center.y() << " " << center.z() << " "
                << covCenter.x() << " " << covCenter.y() << " " << covCenter.z() << " "
                << 2 << "\n";
        }

        const std::set<KeyFrame*> loopEdges = pKF->GetLoopEdges();
        for (KeyFrame* pLoopKF : loopEdges)
        {
            if (!pLoopKF || pLoopKF->isBad() || pLoopKF->mnId <= pKF->mnId)
                continue;

            const Eigen::Vector3f loopCenter = pLoopKF->GetCameraCenter();
            out << center.x() << " " << center.y() << " " << center.z() << " "
                << loopCenter.x() << " " << loopCenter.y() << " " << loopCenter.z() << " "
                << 1 << "\n";
        }

        if (pActiveMap->isImuInitialized() && pKF->mNextKF && !pKF->mNextKF->isBad())
        {
            const Eigen::Vector3f nextCenter = pKF->mNextKF->GetCameraCenter();
            out << center.x() << " " << center.y() << " " << center.z() << " "
                << nextCenter.x() << " " << nextCenter.y() << " " << nextCenter.z() << " "
                << 3 << "\n";
        }
    }

    out.close();
    return std::rename(tempPath.c_str(), mOutputPath.c_str()) == 0;
}

} // namespace ORB_SLAM3
