#include <array>
#include <csignal>
#include <cstdlib>
#include <execinfo.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "monocular_slam/MonocularSession.h"

namespace
{

void CrashSignalHandler(int signum)
{
    void* frames[128];
    const int count = backtrace(frames, 128);
    const char* signal_name =
        (signum == SIGSEGV) ? "SIGSEGV" :
        ((signum == SIGBUS) ? "SIGBUS" :
        ((signum == SIGILL) ? "SIGILL" : "SIGNAL"));
    std::cerr << "\nCrash handler caught " << signal_name << std::endl;
    backtrace_symbols_fd(frames, count, STDERR_FILENO);
    std::_Exit(128 + signum);
}

void InstallCrashHandlers()
{
    std::signal(SIGSEGV, CrashSignalHandler);
    std::signal(SIGBUS, CrashSignalHandler);
    std::signal(SIGILL, CrashSignalHandler);
}

bool LoadIntrinsics(const std::string& settings_path,
                    float& fx,
                    float& fy,
                    float& cx,
                    float& cy)
{
    cv::FileStorage fs(settings_path, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;

    fx = static_cast<float>(fs["Camera1.fx"]);
    fy = static_cast<float>(fs["Camera1.fy"]);
    cx = static_cast<float>(fs["Camera1.cx"]);
    cy = static_cast<float>(fs["Camera1.cy"]);
    return fx > 0.0f && fy > 0.0f;
}

cv::Matx33f RotationFromPose(const std::array<float, 16>& pose_matrix)
{
    return cv::Matx33f(
        pose_matrix[0], pose_matrix[1], pose_matrix[2],
        pose_matrix[4], pose_matrix[5], pose_matrix[6],
        pose_matrix[8], pose_matrix[9], pose_matrix[10]);
}

cv::Vec3f TranslationFromPose(const std::array<float, 16>& pose_matrix)
{
    return cv::Vec3f(pose_matrix[3], pose_matrix[7], pose_matrix[11]);
}

cv::Point3f CameraWorldPositionFromPose(const std::array<float, 16>& pose_matrix)
{
    const cv::Matx33f Rcw = RotationFromPose(pose_matrix);
    const cv::Vec3f tcw = TranslationFromPose(pose_matrix);
    const cv::Vec3f twc = -(Rcw.t() * tcw);
    return cv::Point3f(twc[0], twc[1], twc[2]);
}

std::array<float, 16> MakePoseArray(const cv::Matx33f& rotation, const cv::Vec3f& translation)
{
    return {
        rotation(0, 0), rotation(0, 1), rotation(0, 2), translation[0],
        rotation(1, 0), rotation(1, 1), rotation(1, 2), translation[1],
        rotation(2, 0), rotation(2, 1), rotation(2, 2), translation[2],
        0.f, 0.f, 0.f, 1.f
    };
}

bool ProjectWorldPoint(const monocular_slam::FrameData& frame,
                       const cv::Point3f& world_point,
                       float fx,
                       float fy,
                       float cx,
                       float cy,
                       cv::Point2f& image_point)
{
    if(!frame.has_pose)
        return false;

    const cv::Matx33f Rcw = RotationFromPose(frame.pose_matrix);
    const cv::Vec3f tcw = TranslationFromPose(frame.pose_matrix);
    const cv::Vec3f Xw(world_point.x, world_point.y, world_point.z);
    const cv::Vec3f Xc = Rcw * Xw + tcw;
    if(Xc[2] <= 1e-5f)
        return false;

    image_point.x = fx * Xc[0] / Xc[2] + cx;
    image_point.y = fy * Xc[1] / Xc[2] + cy;
    return std::isfinite(image_point.x) && std::isfinite(image_point.y);
}

std::vector<cv::Point3f> BuildCubeWorldPoints(const monocular_slam::Anchor& anchor, float edge_length)
{
    const cv::Matx33f Rwa = RotationFromPose(anchor.pose_matrix);
    const cv::Vec3f twa = TranslationFromPose(anchor.pose_matrix);
    const float h = edge_length * 0.5f;

    const std::vector<cv::Point3f> local_points = {
        {-h, -h, 0.0f}, { h, -h, 0.0f}, { h,  h, 0.0f}, {-h,  h, 0.0f},
        {-h, -h, edge_length}, { h, -h, edge_length}, { h,  h, edge_length}, {-h,  h, edge_length}
    };

    std::vector<cv::Point3f> world_points;
    world_points.reserve(local_points.size());
    for(const cv::Point3f& point : local_points)
    {
        const cv::Vec3f local(point.x, point.y, point.z);
        const cv::Vec3f world = Rwa * local + twa;
        world_points.emplace_back(world[0], world[1], world[2]);
    }
    return world_points;
}

void DrawAnchorCube(cv::Mat& image,
                    const monocular_slam::FrameData& frame,
                    const monocular_slam::Anchor& anchor,
                    float fx,
                    float fy,
                    float cx,
                    float cy,
                    float edge_length)
{
    const std::vector<cv::Point3f> world_points = BuildCubeWorldPoints(anchor, edge_length);
    std::vector<cv::Point2f> image_points(world_points.size());
    for(std::size_t i = 0; i < world_points.size(); ++i)
    {
        if(!ProjectWorldPoint(frame, world_points[i], fx, fy, cx, cy, image_points[i]))
            return;
    }

    const int edges[][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    for(const auto& edge : edges)
    {
        cv::line(image,
                 image_points[edge[0]],
                 image_points[edge[1]],
                 cv::Scalar(0, 255, 255),
                 2,
                 cv::LINE_AA);
    }

    cv::Point2f label_point = image_points[4];
    cv::putText(image,
                anchor.label.empty() ? "anchor" : anchor.label,
                cv::Point(static_cast<int>(label_point.x), static_cast<int>(label_point.y) - 8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 255),
                2,
                cv::LINE_AA);
}

monocular_slam::Anchor CreateForwardAnchor(monocular_slam::MonocularSession& session,
                                           const monocular_slam::FrameData& frame,
                                           float distance_m,
                                           const std::string& label)
{
    if(!frame.has_pose)
        return monocular_slam::Anchor();

    const cv::Matx33f Rcw = RotationFromPose(frame.pose_matrix);
    const cv::Vec3f tcw = TranslationFromPose(frame.pose_matrix);
    const cv::Matx33f Rwc = Rcw.t();
    const cv::Vec3f twc = -(Rwc * tcw);
    const cv::Vec3f forward_world = Rwc * cv::Vec3f(0.0f, 0.0f, distance_m);
    const cv::Vec3f anchor_translation = twc + forward_world;

    return session.ar.CreateAnchor(MakePoseArray(Rwc, anchor_translation), label);
}

void DrawStatusText(cv::Mat& image,
                    const monocular_slam::FrameData& frame,
                    std::size_t anchor_count)
{
    const std::vector<std::string> lines = {
        "state: " + frame.tracking_state_name,
        "frame: " + std::to_string(frame.frame_id),
        "anchors: " + std::to_string(anchor_count),
        "map pts: " + std::to_string(frame.tracked_map_points_world.size()),
        "A place cube  C clear  Q quit"
    };

    int y = 26;
    for(const std::string& line : lines)
    {
        cv::putText(image, line, cv::Point(14, y), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(image, line, cv::Point(14, y), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y += 24;
    }
}

cv::Mat MakeTopDownMapView(const monocular_slam::FrameData& frame,
                           const std::vector<cv::Point3f>& camera_positions,
                           const std::vector<monocular_slam::Anchor>& anchors,
                           const cv::Size& size)
{
    cv::Mat view(size, CV_8UC3, cv::Scalar(8, 8, 8));
    cv::putText(view, "local map top-down", cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    std::vector<cv::Point3f> world_points;
    world_points.reserve(frame.tracked_map_points_world.size() + camera_positions.size() + anchors.size() + 1);
    for(const monocular_slam::Point3D& point : frame.tracked_map_points_world)
        world_points.emplace_back(point.x, point.y, point.z);
    for(const cv::Point3f& point : camera_positions)
        world_points.push_back(point);
    for(const monocular_slam::Anchor& anchor : anchors)
    {
        if(anchor.valid)
        {
            const cv::Vec3f ta = TranslationFromPose(anchor.pose_matrix);
            world_points.emplace_back(ta[0], ta[1], ta[2]);
        }
    }
    if(frame.has_pose)
        world_points.push_back(CameraWorldPositionFromPose(frame.pose_matrix));

    if(world_points.empty())
        return view;

    float min_x = world_points.front().x;
    float max_x = world_points.front().x;
    float min_z = world_points.front().z;
    float max_z = world_points.front().z;
    for(const cv::Point3f& point : world_points)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    const float pad = 0.3f;
    min_x -= pad;
    max_x += pad;
    min_z -= pad;
    max_z += pad;

    auto project = [&](const cv::Point3f& point) -> cv::Point {
        const float x_norm = (point.x - min_x) / std::max(1e-5f, max_x - min_x);
        const float z_norm = (point.z - min_z) / std::max(1e-5f, max_z - min_z);
        return cv::Point(16 + static_cast<int>(x_norm * (size.width - 32)),
                         size.height - 16 - static_cast<int>(z_norm * (size.height - 32)));
    };

    for(const monocular_slam::Point3D& point : frame.tracked_map_points_world)
        cv::circle(view, project(cv::Point3f(point.x, point.y, point.z)), 1, cv::Scalar(180, 180, 180), -1, cv::LINE_AA);

    for(std::size_t i = 1; i < camera_positions.size(); ++i)
        cv::line(view, project(camera_positions[i - 1]), project(camera_positions[i]), cv::Scalar(90, 90, 255), 1, cv::LINE_AA);

    for(const monocular_slam::Anchor& anchor : anchors)
    {
        if(!anchor.valid)
            continue;
        const cv::Vec3f ta = TranslationFromPose(anchor.pose_matrix);
        const cv::Point p = project(cv::Point3f(ta[0], ta[1], ta[2]));
        cv::circle(view, p, 5, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        cv::putText(view,
                    anchor.label.empty() ? "anchor" : anchor.label,
                    cv::Point(p.x + 8, p.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(0, 255, 255),
                    1,
                    cv::LINE_AA);
    }

    if(frame.has_pose)
    {
        const cv::Point3f camera = CameraWorldPositionFromPose(frame.pose_matrix);
        const cv::Point center = project(camera);
        cv::rectangle(view, cv::Rect(center.x - 5, center.y - 5, 10, 10), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    std::string footer = "tracked world pts: " + std::to_string(frame.tracked_map_points_world.size());
    cv::putText(view, footer, cv::Point(10, size.height - 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    return view;
}

} // namespace

int main(int argc, char** argv)
{
    InstallCrashHandlers();

    if(argc != 4)
    {
        std::cerr << std::endl
                  << "Usage: ./video_ar_session path_to_vocabulary path_to_settings video_source" << std::endl;
        return 1;
    }

    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    if(!LoadIntrinsics(argv[2], fx, fy, cx, cy))
    {
        std::cerr << "Failed to load camera intrinsics from settings." << std::endl;
        return 1;
    }

    monocular_slam::SessionConfig session_config;
    session_config.vocabulary_path = argv[1];
    session_config.settings_path = argv[2];
    session_config.use_viewer = false;
    session_config.sensor_mode = monocular_slam::SensorMode::Monocular;

    monocular_slam::VideoSourceConfig video_config;
    video_config.source = argv[3];
    video_config.realtime_playback = true;

    monocular_slam::MonocularSession session;
    if(!session.Initialize(session_config))
    {
        std::cerr << "Failed to initialize monocular session." << std::endl;
        return 1;
    }
    if(!session.OpenVideo(video_config))
    {
        std::cerr << "Failed to open live video source." << std::endl;
        return 1;
    }

    cv::namedWindow("video_ar_session", cv::WINDOW_NORMAL);
    cv::namedWindow("video_ar_map", cv::WINDOW_NORMAL);
    std::vector<cv::Point3f> camera_positions;

    while(true)
    {
        if(!session.StepVideoFrame())
        {
            session.FinalizeVideo();
            break;
        }

        const monocular_slam::SessionSnapshot snapshot = session.GetSnapshot();
        const monocular_slam::FrameData& frame = snapshot.current_frame;
        if(frame.frame_bgr.empty())
            continue;

        if(frame.has_pose)
        {
            camera_positions.push_back(CameraWorldPositionFromPose(frame.pose_matrix));
            if(camera_positions.size() > 3000)
                camera_positions.erase(camera_positions.begin(),
                                      camera_positions.begin() + static_cast<long>(camera_positions.size() - 3000));
        }

        cv::Mat display = frame.frame_bgr.clone();
        const std::vector<monocular_slam::Anchor> anchors = session.ar.GetAnchors();
        for(const monocular_slam::Anchor& anchor : anchors)
        {
            if(anchor.valid)
                DrawAnchorCube(display, frame, anchor, fx, fy, cx, cy, 0.12f);
        }
        DrawStatusText(display, frame, anchors.size());
        cv::imshow("video_ar_session", display);
        cv::imshow("video_ar_map", MakeTopDownMapView(frame, camera_positions, anchors, cv::Size(500, 500)));

        const int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q')
        {
            session.FinalizeVideo();
            break;
        }
        if(key == 'a' || key == 'A')
        {
            const monocular_slam::Anchor anchor = CreateForwardAnchor(session, frame, 0.75f, "cube");
            if(anchor.valid)
                std::cout << "Created anchor " << anchor.id << " at frame " << anchor.source_frame_id << std::endl;
        }
        if(key == 'c' || key == 'C')
        {
            session.ar.ClearAnchors();
            std::cout << "Cleared anchors" << std::endl;
        }
    }

    session.Wait();
    cv::destroyAllWindows();
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(0);
}
