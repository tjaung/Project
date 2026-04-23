#include "monocular_slam/MonocularSession.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "monocular_slam/MonocularPipeline.h"

namespace monocular_slam
{

namespace
{

std::string Basename(const std::string& path)
{
    const std::size_t pos = path.find_last_of("/\\");
    if(pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
}

void LoadTumImages(const std::string& file_path,
                   std::vector<std::string>& image_filenames,
                   std::vector<double>& timestamps)
{
    std::ifstream file(file_path.c_str());

    std::string line;
    std::getline(file, line);
    std::getline(file, line);
    std::getline(file, line);

    while(std::getline(file, line))
    {
        if(line.empty())
            continue;

        std::stringstream stream(line);
        double timestamp = 0.0;
        std::string image_name;
        stream >> timestamp >> image_name;
        if(image_name.empty())
            continue;

        timestamps.push_back(timestamp);
        image_filenames.push_back(image_name);
    }
}

bool IsIntegerString(const std::string& text)
{
    if(text.empty())
        return false;
    std::size_t start = 0;
    if(text[0] == '-' || text[0] == '+')
        start = 1;
    if(start >= text.size())
        return false;
    for(std::size_t i = start; i < text.size(); ++i)
    {
        if(text[i] < '0' || text[i] > '9')
            return false;
    }
    return true;
}

std::string NormalizeVideoSource(const std::string& source)
{
    if(source.empty() || IsIntegerString(source))
        return source;

    if(source.find("://") != std::string::npos)
        return source;

    const bool looks_like_host_port =
        source.find(':') != std::string::npos &&
        source.find('/') == std::string::npos &&
        source.find('\\') == std::string::npos;

    if(looks_like_host_port)
        return "http://" + source + "/video";

    return source;
}

RunMetrics BuildRunMetrics(const std::vector<float>& times_track,
                           const DatasetRunConfig& config)
{
    RunMetrics metrics;
    metrics.frames_processed = times_track.size();
    metrics.trajectory_path = config.trajectory_path;
    metrics.keyframe_trajectory_path = config.keyframe_trajectory_path;
    metrics.metrics_csv_path = config.metrics_csv_path;
    metrics.dataset_name = Basename(config.dataset_path);

    if(times_track.empty())
        return metrics;

    std::vector<float> sorted_times = times_track;
    std::sort(sorted_times.begin(), sorted_times.end());
    float total_time = 0.0f;
    for(float t : sorted_times)
        total_time += t;
    metrics.median_tracking_time_sec = sorted_times[sorted_times.size() / 2];
    metrics.mean_tracking_time_sec = total_time / static_cast<float>(sorted_times.size());
    return metrics;
}

} // namespace

struct MonocularSession::Impl
{
    mutable std::mutex mutex;
    std::condition_variable pause_cv;
    SessionConfig session_config;
    MonocularPipeline pipeline;
    std::thread worker;
    RunnerState state = RunnerState::Idle;
    bool stop_requested = false;
    bool pause_requested = false;
    bool initialized = false;
    RunMetrics metrics;
    SessionSnapshot snapshot;
    DatasetRunConfig dataset_config;
    std::vector<std::string> dataset_image_filenames;
    std::vector<double> dataset_timestamps;
    std::vector<float> dataset_times_track;
    std::size_t dataset_index = 0;
    bool dataset_open = false;
    VideoSourceConfig video_config;
    cv::VideoCapture video_capture;
    std::vector<float> video_times_track;
    double video_fallback_timestamp = 0.0;
    double video_fps = 30.0;
    bool video_open = false;
    std::vector<Anchor> anchors;
    std::uint64_t next_anchor_id = 1;

    void JoinWorkerIfNeeded()
    {
        if(worker.joinable())
            worker.join();
    }

    void ResetDatasetState()
    {
        dataset_config = DatasetRunConfig();
        dataset_image_filenames.clear();
        dataset_timestamps.clear();
        dataset_times_track.clear();
        dataset_index = 0;
        dataset_open = false;
    }

    void ResetVideoState()
    {
        video_config = VideoSourceConfig();
        if(video_capture.isOpened())
            video_capture.release();
        video_times_track.clear();
        video_fallback_timestamp = 0.0;
        video_fps = 30.0;
        video_open = false;
    }
};

MonocularSession::ARApi::ARApi()
    : mOwner(nullptr)
{
}

MonocularSession::ARApi::ARApi(MonocularSession* owner)
    : mOwner(owner)
{
}

MonocularSession::MonocularSession()
    : ar(this)
    , mImpl(new Impl())
{
}

MonocularSession::~MonocularSession()
{
    Shutdown();
}

bool MonocularSession::Initialize(const SessionConfig& config)
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(mImpl->initialized)
        return true;

    if(!mImpl->pipeline.Initialize(config))
        return false;

    mImpl->session_config = config;
    mImpl->initialized = true;
    mImpl->state = RunnerState::Idle;
    mImpl->metrics = RunMetrics();
    mImpl->snapshot = SessionSnapshot();
    mImpl->snapshot.initialized = true;
    mImpl->ResetDatasetState();
    mImpl->ResetVideoState();
    mImpl->anchors.clear();
    mImpl->next_anchor_id = 1;
    return true;
}

void MonocularSession::Shutdown()
{
    Stop();
    mImpl->JoinWorkerIfNeeded();

    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(mImpl->initialized)
    {
        mImpl->pipeline.Shutdown();
        mImpl->initialized = false;
    }
    mImpl->state = RunnerState::Idle;
    mImpl->snapshot = SessionSnapshot();
    mImpl->ResetDatasetState();
    mImpl->ResetVideoState();
    mImpl->anchors.clear();
    mImpl->next_anchor_id = 1;
}

bool MonocularSession::StartVideo(const VideoSourceConfig& config)
{
    {
        std::lock_guard<std::mutex> lock(mImpl->mutex);
        if(!mImpl->initialized || mImpl->worker.joinable())
            return false;
        mImpl->stop_requested = false;
        mImpl->pause_requested = false;
        mImpl->metrics = RunMetrics();
        mImpl->state = RunnerState::Running;
        mImpl->snapshot = SessionSnapshot();
        mImpl->snapshot.initialized = true;
        mImpl->snapshot.running = true;
        mImpl->snapshot.runner_state = RunnerState::Running;
    }

    mImpl->worker = std::thread([this, config]() {
        cv::VideoCapture cap;
        const std::string normalized_source = NormalizeVideoSource(config.source);
        if(IsIntegerString(normalized_source))
            cap.open(std::atoi(normalized_source.c_str()));
        else
            cap.open(normalized_source);

        if(!cap.isOpened())
        {
            std::lock_guard<std::mutex> lock(mImpl->mutex);
            mImpl->state = RunnerState::Failed;
            return;
        }

        std::vector<float> times_track;
        double fallback_timestamp = 0.0;
        double fps = cap.get(cv::CAP_PROP_FPS);
        if(fps <= 0.0)
            fps = 30.0;

        while(true)
        {
            {
                std::unique_lock<std::mutex> lock(mImpl->mutex);
                mImpl->pause_cv.wait(lock, [this]() { return !mImpl->pause_requested || mImpl->stop_requested; });
                if(mImpl->stop_requested)
                    break;
            }

            cv::Mat frame;
            if(!cap.read(frame) || frame.empty())
                break;

            const auto t1 = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(mImpl->mutex);
                mImpl->pipeline.ProcessFrame(frame, fallback_timestamp);
                mImpl->snapshot.initialized = mImpl->initialized;
                mImpl->snapshot.running = true;
                mImpl->snapshot.paused = false;
                mImpl->snapshot.runner_state = mImpl->state;
                mImpl->snapshot.current_frame = mImpl->pipeline.GetCurrentFrameData();
                mImpl->snapshot.previous_frame = mImpl->pipeline.GetPreviousFrameData();
                mImpl->snapshot.metrics = mImpl->metrics;
            }
            const auto t2 = std::chrono::steady_clock::now();
            const float track_seconds = static_cast<float>(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
            times_track.push_back(track_seconds);

            fallback_timestamp += 1.0 / fps;

            if(config.realtime_playback)
            {
                const double frame_period = 1.0 / fps;
                if(track_seconds < frame_period)
                    std::this_thread::sleep_for(std::chrono::duration<double>(frame_period - track_seconds));
            }
        }

        std::lock_guard<std::mutex> lock(mImpl->mutex);
        mImpl->metrics.frames_processed = times_track.size();
        if(!times_track.empty())
        {
            std::sort(times_track.begin(), times_track.end());
            float total_time = 0.0f;
            for(float t : times_track)
                total_time += t;
            mImpl->metrics.median_tracking_time_sec = times_track[times_track.size() / 2];
            mImpl->metrics.mean_tracking_time_sec = total_time / static_cast<float>(times_track.size());
        }
        if(mImpl->state != RunnerState::Failed)
            mImpl->state = RunnerState::Finished;
        mImpl->snapshot.initialized = mImpl->initialized;
        mImpl->snapshot.running = false;
        mImpl->snapshot.paused = false;
        mImpl->snapshot.runner_state = mImpl->state;
        mImpl->snapshot.metrics = mImpl->metrics;
    });

    return true;
}

bool MonocularSession::OpenVideo(const VideoSourceConfig& config)
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->initialized || mImpl->worker.joinable())
        return false;

    const std::string normalized_source = NormalizeVideoSource(config.source);
    cv::VideoCapture cap;
    if(IsIntegerString(normalized_source))
        cap.open(std::atoi(normalized_source.c_str()));
    else
        cap.open(normalized_source);

    if(!cap.isOpened())
        return false;

    mImpl->ResetVideoState();
    mImpl->video_capture = std::move(cap);
    mImpl->video_config = config;
    mImpl->video_fps = mImpl->video_capture.get(cv::CAP_PROP_FPS);
    if(mImpl->video_fps <= 0.0)
        mImpl->video_fps = 30.0;
    mImpl->video_fallback_timestamp = 0.0;
    mImpl->video_open = true;
    mImpl->metrics = RunMetrics();
    mImpl->state = RunnerState::Running;
    mImpl->snapshot = SessionSnapshot();
    mImpl->snapshot.initialized = true;
    mImpl->snapshot.running = true;
    mImpl->snapshot.runner_state = RunnerState::Running;
    return true;
}

bool MonocularSession::StepVideoFrame()
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->initialized || !mImpl->video_open || !mImpl->video_capture.isOpened())
        return false;

    cv::Mat frame;
    if(!mImpl->video_capture.read(frame) || frame.empty())
        return false;

    const auto t1 = std::chrono::steady_clock::now();
    mImpl->pipeline.ProcessFrame(frame, mImpl->video_fallback_timestamp);
    const auto t2 = std::chrono::steady_clock::now();
    const float track_seconds = static_cast<float>(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
    mImpl->video_times_track.push_back(track_seconds);

    mImpl->snapshot.initialized = mImpl->initialized;
    mImpl->snapshot.running = true;
    mImpl->snapshot.paused = false;
    mImpl->snapshot.runner_state = mImpl->state;
    mImpl->snapshot.current_frame = mImpl->pipeline.GetCurrentFrameData();
    mImpl->snapshot.previous_frame = mImpl->pipeline.GetPreviousFrameData();
    mImpl->snapshot.metrics = mImpl->metrics;

    mImpl->video_fallback_timestamp += 1.0 / mImpl->video_fps;
    return true;
}

void MonocularSession::FinalizeVideo()
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->video_open)
        return;

    mImpl->metrics.frames_processed = mImpl->video_times_track.size();
    if(!mImpl->video_times_track.empty())
    {
        std::vector<float> sorted_times = mImpl->video_times_track;
        std::sort(sorted_times.begin(), sorted_times.end());
        float total_time = 0.0f;
        for(float t : sorted_times)
            total_time += t;
        mImpl->metrics.median_tracking_time_sec = sorted_times[sorted_times.size() / 2];
        mImpl->metrics.mean_tracking_time_sec = total_time / static_cast<float>(sorted_times.size());
    }

    mImpl->state = RunnerState::Finished;
    mImpl->snapshot.initialized = mImpl->initialized;
    mImpl->snapshot.running = false;
    mImpl->snapshot.paused = false;
    mImpl->snapshot.runner_state = mImpl->state;
    mImpl->snapshot.metrics = mImpl->metrics;
    mImpl->ResetVideoState();
}

bool MonocularSession::StartDatasetRun(const DatasetRunConfig& config)
{
    {
        std::lock_guard<std::mutex> lock(mImpl->mutex);
        if(!mImpl->initialized || mImpl->worker.joinable())
            return false;
        mImpl->stop_requested = false;
        mImpl->pause_requested = false;
        mImpl->metrics = RunMetrics();
        mImpl->state = RunnerState::Running;
        mImpl->snapshot = SessionSnapshot();
        mImpl->snapshot.initialized = true;
        mImpl->snapshot.running = true;
        mImpl->snapshot.runner_state = RunnerState::Running;
    }

    mImpl->worker = std::thread([this, config]() {
        std::vector<std::string> image_filenames;
        std::vector<double> timestamps;
        LoadTumImages(config.dataset_path + "/rgb.txt", image_filenames, timestamps);
        if(image_filenames.empty())
        {
            std::lock_guard<std::mutex> lock(mImpl->mutex);
            mImpl->state = RunnerState::Failed;
            return;
        }

        std::vector<float> times_track;
        times_track.reserve(image_filenames.size());

        for(std::size_t i = 0; i < image_filenames.size(); ++i)
        {
            {
                std::unique_lock<std::mutex> lock(mImpl->mutex);
                mImpl->pause_cv.wait(lock, [this]() { return !mImpl->pause_requested || mImpl->stop_requested; });
                if(mImpl->stop_requested)
                    break;
            }

            const std::string frame_path = config.dataset_path + "/" + image_filenames[i];
            cv::Mat frame = cv::imread(frame_path, cv::IMREAD_UNCHANGED);
            if(frame.empty())
                continue;

            const auto t1 = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(mImpl->mutex);
                mImpl->pipeline.ProcessFrame(frame, timestamps[i], image_filenames[i]);
                mImpl->snapshot.initialized = mImpl->initialized;
                mImpl->snapshot.running = true;
                mImpl->snapshot.paused = false;
                mImpl->snapshot.runner_state = mImpl->state;
                mImpl->snapshot.current_frame = mImpl->pipeline.GetCurrentFrameData();
                mImpl->snapshot.previous_frame = mImpl->pipeline.GetPreviousFrameData();
                mImpl->snapshot.metrics = mImpl->metrics;
            }
            const auto t2 = std::chrono::steady_clock::now();
            const float track_seconds = static_cast<float>(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
            times_track.push_back(track_seconds);

            if(config.realtime_playback)
            {
                double wait_seconds = 0.0;
                if(i + 1 < timestamps.size())
                    wait_seconds = timestamps[i + 1] - timestamps[i];
                if(track_seconds < wait_seconds)
                    std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds - track_seconds));
            }
        }

        RunMetrics metrics = BuildRunMetrics(times_track, config);
        if(config.save_trajectories)
        {
            {
                std::lock_guard<std::mutex> lock(mImpl->mutex);
                mImpl->pipeline.SaveTrajectoryEuRoC(config.trajectory_path);
                mImpl->pipeline.SaveKeyFrameTrajectoryTUM(config.keyframe_trajectory_path);
            }
            metrics.trajectories_saved = true;
        }

        const std::string groundtruth_file = config.dataset_path + "/groundtruth.txt";
        std::ifstream gt(groundtruth_file.c_str());
        if(config.evaluate_metrics && gt.good())
        {
            metrics.metric_evaluation_attempted = true;
            std::ostringstream cmd;
            cmd << "python3 evaluation/evaluate_tum_metrics.py "
                << "\"" << groundtruth_file << "\" "
                << "\"" << config.trajectory_path << "\" "
                << "--tracking-time-sec "
                << std::fixed << std::setprecision(9) << metrics.mean_tracking_time_sec << " "
                << "--median-tracking-time-sec "
                << std::fixed << std::setprecision(9) << metrics.median_tracking_time_sec << " "
                << "--dataset-name "
                << "\"" << metrics.dataset_name << "\" "
                << "--csv "
                << "\"" << config.metrics_csv_path << "\"";

            const int eval_status = std::system(cmd.str().c_str());
            metrics.metric_evaluator_exit_code = eval_status;
            metrics.metric_evaluation_succeeded = (eval_status == 0);
        }

        std::lock_guard<std::mutex> lock(mImpl->mutex);
        mImpl->metrics = metrics;
        if(mImpl->state != RunnerState::Failed)
            mImpl->state = RunnerState::Finished;
        mImpl->snapshot.initialized = mImpl->initialized;
        mImpl->snapshot.running = false;
        mImpl->snapshot.paused = false;
        mImpl->snapshot.runner_state = mImpl->state;
        mImpl->snapshot.metrics = mImpl->metrics;
    });

    return true;
}

bool MonocularSession::OpenDatasetRun(const DatasetRunConfig& config)
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->initialized || mImpl->worker.joinable())
        return false;

    std::vector<std::string> image_filenames;
    std::vector<double> timestamps;
    LoadTumImages(config.dataset_path + "/rgb.txt", image_filenames, timestamps);
    if(image_filenames.empty())
        return false;

    mImpl->ResetDatasetState();
    mImpl->dataset_config = config;
    mImpl->dataset_image_filenames = std::move(image_filenames);
    mImpl->dataset_timestamps = std::move(timestamps);
    mImpl->dataset_times_track.reserve(mImpl->dataset_image_filenames.size());
    mImpl->dataset_index = 0;
    mImpl->dataset_open = true;
    mImpl->metrics = RunMetrics();
    mImpl->state = RunnerState::Running;
    mImpl->snapshot = SessionSnapshot();
    mImpl->snapshot.initialized = true;
    mImpl->snapshot.running = true;
    mImpl->snapshot.runner_state = RunnerState::Running;
    return true;
}

bool MonocularSession::StepDatasetFrame()
{
    bool finalize_after = false;
    {
        std::lock_guard<std::mutex> lock(mImpl->mutex);
        if(!mImpl->initialized || !mImpl->dataset_open)
            return false;

        if(mImpl->dataset_index >= mImpl->dataset_image_filenames.size())
            return false;

        const std::string frame_path = mImpl->dataset_config.dataset_path + "/" + mImpl->dataset_image_filenames[mImpl->dataset_index];
        cv::Mat frame = cv::imread(frame_path, cv::IMREAD_UNCHANGED);
        if(frame.empty())
        {
            ++mImpl->dataset_index;
            finalize_after = (mImpl->dataset_index >= mImpl->dataset_image_filenames.size());
        }
        else
        {
            const auto t1 = std::chrono::steady_clock::now();
            mImpl->pipeline.ProcessFrame(frame,
                                         mImpl->dataset_timestamps[mImpl->dataset_index],
                                         mImpl->dataset_image_filenames[mImpl->dataset_index]);
            const auto t2 = std::chrono::steady_clock::now();
            const float track_seconds = static_cast<float>(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
            mImpl->dataset_times_track.push_back(track_seconds);

            mImpl->snapshot.initialized = mImpl->initialized;
            mImpl->snapshot.running = true;
            mImpl->snapshot.paused = false;
            mImpl->snapshot.runner_state = mImpl->state;
            mImpl->snapshot.current_frame = mImpl->pipeline.GetCurrentFrameData();
            mImpl->snapshot.previous_frame = mImpl->pipeline.GetPreviousFrameData();
            mImpl->snapshot.metrics = mImpl->metrics;

            ++mImpl->dataset_index;
            finalize_after = (mImpl->dataset_index >= mImpl->dataset_image_filenames.size());
        }
    }

    if(finalize_after)
        FinalizeDatasetRun();

    return true;
}

void MonocularSession::FinalizeDatasetRun()
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(!mImpl->dataset_open)
        return;

    RunMetrics metrics = BuildRunMetrics(mImpl->dataset_times_track, mImpl->dataset_config);
    if(mImpl->dataset_config.save_trajectories)
    {
        try
        {
            mImpl->pipeline.SaveTrajectoryEuRoC(mImpl->dataset_config.trajectory_path);
            mImpl->pipeline.SaveKeyFrameTrajectoryTUM(mImpl->dataset_config.keyframe_trajectory_path);
            metrics.trajectories_saved = true;
        }
        catch(const std::exception& e)
        {
            std::cerr << "Trajectory export failed: " << e.what() << std::endl;
            metrics.trajectories_saved = false;
        }
    }

    const std::string groundtruth_file = mImpl->dataset_config.dataset_path + "/groundtruth.txt";
    std::ifstream gt(groundtruth_file.c_str());
    if(mImpl->dataset_config.evaluate_metrics && gt.good())
    {
        metrics.metric_evaluation_attempted = true;
        std::ostringstream cmd;
        cmd << "python3 evaluation/evaluate_tum_metrics.py "
            << "\"" << groundtruth_file << "\" "
            << "\"" << mImpl->dataset_config.trajectory_path << "\" "
            << "--tracking-time-sec "
            << std::fixed << std::setprecision(9) << metrics.mean_tracking_time_sec << " "
            << "--median-tracking-time-sec "
            << std::fixed << std::setprecision(9) << metrics.median_tracking_time_sec << " "
            << "--dataset-name "
            << "\"" << metrics.dataset_name << "\" "
            << "--csv "
            << "\"" << mImpl->dataset_config.metrics_csv_path << "\"";

        const int eval_status = std::system(cmd.str().c_str());
        metrics.metric_evaluator_exit_code = eval_status;
        metrics.metric_evaluation_succeeded = (eval_status == 0);
    }

    mImpl->metrics = metrics;
    mImpl->state = RunnerState::Finished;
    mImpl->snapshot.initialized = mImpl->initialized;
    mImpl->snapshot.running = false;
    mImpl->snapshot.paused = false;
    mImpl->snapshot.runner_state = mImpl->state;
    mImpl->snapshot.metrics = mImpl->metrics;
    mImpl->dataset_open = false;
}

void MonocularSession::Stop()
{
    {
        std::lock_guard<std::mutex> lock(mImpl->mutex);
        mImpl->stop_requested = true;
        mImpl->pause_requested = false;
        mImpl->snapshot.running = false;
        mImpl->snapshot.paused = false;
    }
    mImpl->pause_cv.notify_all();
    mImpl->JoinWorkerIfNeeded();
}

void MonocularSession::Pause()
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    if(mImpl->state == RunnerState::Running)
    {
        mImpl->pause_requested = true;
        mImpl->state = RunnerState::Paused;
        mImpl->snapshot.running = false;
        mImpl->snapshot.paused = true;
        mImpl->snapshot.runner_state = mImpl->state;
    }
}

void MonocularSession::Resume()
{
    {
        std::lock_guard<std::mutex> lock(mImpl->mutex);
        if(mImpl->state == RunnerState::Paused)
        {
            mImpl->pause_requested = false;
            mImpl->state = RunnerState::Running;
            mImpl->snapshot.running = true;
            mImpl->snapshot.paused = false;
            mImpl->snapshot.runner_state = mImpl->state;
        }
    }
    mImpl->pause_cv.notify_all();
}

void MonocularSession::Wait()
{
    mImpl->JoinWorkerIfNeeded();
}

bool MonocularSession::IsInitialized() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->initialized;
}

bool MonocularSession::IsRunning() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->state == RunnerState::Running;
}

bool MonocularSession::IsPaused() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->state == RunnerState::Paused;
}

RunnerState MonocularSession::GetRunnerState() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->state;
}

SessionSnapshot MonocularSession::GetSnapshot() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->snapshot;
}

FrameData MonocularSession::GetCurrentFrameData() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->pipeline.GetCurrentFrameData();
}

FrameData MonocularSession::GetPreviousFrameData() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->pipeline.GetPreviousFrameData();
}

RunMetrics MonocularSession::GetRunMetrics() const
{
    std::lock_guard<std::mutex> lock(mImpl->mutex);
    return mImpl->metrics;
}

Anchor MonocularSession::ARApi::CreateAnchorAtCurrentPose(const std::string& label)
{
    if(!mOwner)
        return Anchor();

    std::lock_guard<std::mutex> lock(mOwner->mImpl->mutex);
    const FrameData& frame = mOwner->mImpl->snapshot.current_frame;
    if(!mOwner->mImpl->initialized || !frame.has_pose)
        return Anchor();

    Anchor anchor;
    anchor.id = mOwner->mImpl->next_anchor_id++;
    anchor.valid = true;
    anchor.source_frame_id = frame.frame_id;
    anchor.timestamp = frame.timestamp;
    anchor.label = label;
    anchor.pose_matrix = frame.pose_matrix;
    mOwner->mImpl->anchors.push_back(anchor);
    return anchor;
}

Anchor MonocularSession::ARApi::CreateAnchor(const std::array<float, 16>& world_pose, const std::string& label)
{
    if(!mOwner)
        return Anchor();

    std::lock_guard<std::mutex> lock(mOwner->mImpl->mutex);
    if(!mOwner->mImpl->initialized)
        return Anchor();

    Anchor anchor;
    anchor.id = mOwner->mImpl->next_anchor_id++;
    anchor.valid = true;
    anchor.source_frame_id = mOwner->mImpl->snapshot.current_frame.frame_id;
    anchor.timestamp = mOwner->mImpl->snapshot.current_frame.timestamp;
    anchor.label = label;
    anchor.pose_matrix = world_pose;
    mOwner->mImpl->anchors.push_back(anchor);
    return anchor;
}

std::vector<Anchor> MonocularSession::ARApi::GetAnchors() const
{
    if(!mOwner)
        return {};

    std::lock_guard<std::mutex> lock(mOwner->mImpl->mutex);
    return mOwner->mImpl->anchors;
}

bool MonocularSession::ARApi::RemoveAnchor(std::uint64_t anchor_id)
{
    if(!mOwner)
        return false;

    std::lock_guard<std::mutex> lock(mOwner->mImpl->mutex);
    auto& anchors = mOwner->mImpl->anchors;
    const auto it = std::remove_if(anchors.begin(), anchors.end(), [anchor_id](const Anchor& anchor) {
        return anchor.id == anchor_id;
    });
    if(it == anchors.end())
        return false;
    anchors.erase(it, anchors.end());
    return true;
}

void MonocularSession::ARApi::ClearAnchors()
{
    if(!mOwner)
        return;

    std::lock_guard<std::mutex> lock(mOwner->mImpl->mutex);
    mOwner->mImpl->anchors.clear();
}

} // namespace monocular_slam
