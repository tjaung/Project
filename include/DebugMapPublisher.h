#ifndef DEBUG_MAP_PUBLISHER_H
#define DEBUG_MAP_PUBLISHER_H

#include "Atlas.h"
#include "MapDrawer.h"

#include <mutex>
#include <string>

namespace ORB_SLAM3
{

class FrameDrawer;

class DebugMapPublisher
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DebugMapPublisher(Atlas* pAtlas, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const std::string& outputPath, int intervalMs = 100);

    void Run();
    void RequestFinish();
    bool isFinished();

private:
    struct CameraSample
    {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f forward = Eigen::Vector3f::UnitZ();
        Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    };

    CameraSample SampleCamera() const;
    bool WriteFrameImage(const std::string& frameImagePath);
    bool WriteSnapshot();

    Atlas* mpAtlas;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    std::string mOutputPath;
    int mIntervalMs;

    bool mbFinishRequested;
    bool mbFinished;
    mutable std::mutex mMutexFinish;
};

} // namespace ORB_SLAM3

#endif
