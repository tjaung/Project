#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cstdlib>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "monocular_slam/MonocularPipeline.h"

namespace
{

cv::Mat MakePlaceholderPreview(const cv::Size& size, const std::string& label)
{
    cv::Mat preview(size, CV_8UC3, cv::Scalar(24, 24, 24));
    cv::putText(preview,
                label,
                cv::Point(14, std::max(32, size.height / 2)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(220, 220, 220),
                1,
                cv::LINE_AA);
    return preview;
}

cv::Point3f CameraWorldPositionFromPose(const std::array<float, 16>& pose_matrix)
{
    cv::Matx33f rotation(
        pose_matrix[0], pose_matrix[1], pose_matrix[2],
        pose_matrix[4], pose_matrix[5], pose_matrix[6],
        pose_matrix[8], pose_matrix[9], pose_matrix[10]);
    cv::Vec3f translation(pose_matrix[3], pose_matrix[7], pose_matrix[11]);
    cv::Vec3f camera = -(rotation.t() * translation);
    return cv::Point3f(camera[0], camera[1], camera[2]);
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

const cv::Mat& PickLatestAvailableMat(const cv::Mat& current,
                                      const cv::Mat& previous)
{
    if(!current.empty())
        return current;
    return previous;
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

std::string Basename(const std::string& path)
{
    const std::size_t pos = path.find_last_of("/\\");
    if(pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
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

cv::Mat MakeDepthPreview(const cv::Mat& depth)
{
    if(depth.empty())
        return cv::Mat();

    cv::Mat depth_float;
    if(depth.type() == CV_32F)
        depth_float = depth;
    else
        depth.convertTo(depth_float, CV_32F);

    cv::Mat valid_mask = depth_float > 1e-6f;
    if(cv::countNonZero(valid_mask) == 0)
        return MakePlaceholderPreview(depth.size(), "no depth");

    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(depth_float, &min_value, &max_value, nullptr, nullptr, valid_mask);

    if(max_value <= min_value)
        return MakePlaceholderPreview(depth.size(), "flat depth");

    cv::Mat normalized;
    depth_float.convertTo(normalized, CV_8U, 255.0 / (max_value - min_value), -255.0 * min_value / (max_value - min_value));
    normalized.setTo(0, ~valid_mask);

    cv::Mat colored;
    cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
    return colored;
}

cv::Mat MakeMaskPreview(const cv::Mat& mask, const monocular_slam::MaskPropagationData& propagation)
{
    if(mask.empty())
        return cv::Mat();

    cv::Mat mask_binary;
    if(mask.type() == CV_8UC1)
        cv::compare(mask, 0, mask_binary, cv::CMP_GT);
    else
        cv::cvtColor(mask, mask_binary, cv::COLOR_BGR2GRAY);

    cv::Mat preview(mask.size(), CV_8UC3, cv::Scalar(18, 18, 18));
    preview.setTo(cv::Scalar(0, 0, 220), mask_binary);

    for(const monocular_slam::Point2D& point : propagation.seed_points)
        cv::circle(preview, cv::Point2f(point.x, point.y), 2, cv::Scalar(255, 0, 255), -1, cv::LINE_AA);

    for(const monocular_slam::Point2D& point : propagation.flowed_points)
        cv::circle(preview, cv::Point2f(point.x, point.y), 2, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);

    if(cv::countNonZero(mask_binary) == 0)
        return MakePlaceholderPreview(mask.size(), "no dynamic mask");

    return preview;
}

void DrawPoints(cv::Mat& image,
                const std::vector<monocular_slam::KeyPointObservation>& points,
                const cv::Scalar& color,
                int radius,
                int thickness,
                std::size_t stride = 1)
{
    if(stride == 0)
        stride = 1;

    for(std::size_t i = 0; i < points.size(); i += stride)
    {
        const monocular_slam::KeyPointObservation& point = points[i];
        cv::circle(image, cv::Point2f(point.x, point.y), radius, color, thickness, cv::LINE_AA);
    }
}

void DrawFlowTracks(cv::Mat& image,
                    const std::vector<monocular_slam::OpticalFlowTrack>& tracks,
                    std::size_t stride = 1)
{
    if(stride == 0)
        stride = 1;

    for(std::size_t i = 0; i < tracks.size(); i += stride)
    {
        const monocular_slam::OpticalFlowTrack& track = tracks[i];
        const cv::Scalar color = track.inlier ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 140, 255);
        cv::Point2f previous(track.previous_point.x, track.previous_point.y);
        cv::Point2f current(track.current_point.x, track.current_point.y);
        cv::line(image, previous, current, color, 1, cv::LINE_AA);
        cv::circle(image, current, 2, color, -1, cv::LINE_AA);
    }
}

void DrawPropagationPoints(cv::Mat& image,
                           const monocular_slam::MaskPropagationData& propagation)
{
    for(const monocular_slam::Point2D& point : propagation.seed_points)
        cv::circle(image, cv::Point2f(point.x, point.y), 1, cv::Scalar(255, 0, 255), -1, cv::LINE_AA);

    for(const monocular_slam::Point2D& point : propagation.flowed_points)
        cv::circle(image, cv::Point2f(point.x, point.y), 1, cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
}

void DrawDetections(cv::Mat& image,
                    const std::vector<monocular_slam::DetectionBox>& detections)
{
    for(const monocular_slam::DetectionBox& detection : detections)
    {
        cv::rectangle(image,
                      cv::Rect(detection.x, detection.y, detection.width, detection.height),
                      cv::Scalar(0, 255, 0),
                      2,
                      cv::LINE_AA);

        std::ostringstream label;
        label << detection.class_name << " " << std::fixed << std::setprecision(2) << detection.confidence;
        cv::putText(image,
                    label.str(),
                    cv::Point(detection.x, std::max(16, detection.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(0, 255, 0),
                    1,
                    cv::LINE_AA);
    }
}

void DrawTextBlock(cv::Mat& image,
                   const monocular_slam::FrameData& current,
                   const monocular_slam::FrameData& previous)
{
    std::vector<std::string> lines;
    {
        std::ostringstream line;
        line << "frame " << current.frame_id << "  state " << current.tracking_state_name;
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "prev frame " << previous.frame_id << "  prev state " << previous.tracking_state_name;
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "raw ORB " << current.raw_orb_keypoints.size()
             << "  filtered ORB " << current.filtered_orb_keypoints.size()
             << "  tracked " << current.tracked_keypoints.size();
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "map pts " << current.tracked_map_point_count
             << "  active map " << current.active_map_point_count
             << " pts / " << current.active_map_keyframe_count << " kfs";
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "YOLO boxes " << current.yolo_detections.size()
             << "  OF tracks " << current.optical_flow_tracks.size()
             << "  prop flow " << current.mask_propagation.flowed_points.size();
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "depth now " << (current.has_estimated_depth ? "yes" : "no")
             << "  prev " << (previous.has_estimated_depth ? "yes" : "no")
             << "  mask now " << (current.has_dynamic_mask ? "yes" : "no")
             << "  prev " << (previous.has_dynamic_mask ? "yes" : "no");
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "depth src " << current.depth_source_frame_id
             << "  yolo src " << current.yolo_source_frame_id
             << "  mask src " << current.mask_source_frame_id;
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "scene pts " << current.tracked_map_point_count;
        lines.push_back(line.str());
    }
    {
        std::ostringstream line;
        line << "KF requested " << (current.requested_new_keyframe ? "yes" : "no")
             << "  frame is KF " << (current.is_keyframe ? "yes" : "no")
             << "  OF used " << (current.used_optical_flow ? "yes" : "no");
        lines.push_back(line.str());
    }

    int y = 24;
    for(const std::string& line : lines)
    {
        cv::putText(image, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(image, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y += 22;
    }
}

cv::Mat MakeTopDownView(const cv::Size& size,
                        const std::vector<cv::Point3f>& camera_positions,
                        const cv::Point3f& current_camera)
{
    cv::Mat view(size, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(view, "camera top-down", cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    if(camera_positions.empty())
        return view;

    float min_x = camera_positions.front().x;
    float max_x = camera_positions.front().x;
    float min_z = camera_positions.front().z;
    float max_z = camera_positions.front().z;
    for(const cv::Point3f& point : camera_positions)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    min_x = std::min(min_x, current_camera.x);
    max_x = std::max(max_x, current_camera.x);
    min_z = std::min(min_z, current_camera.z);
    max_z = std::max(max_z, current_camera.z);

    const float pad = 0.25f;
    min_x -= pad; max_x += pad;
    min_z -= pad; max_z += pad;

    auto project = [&](const cv::Point3f& point) -> cv::Point {
        const float x_norm = (point.x - min_x) / std::max(1e-5f, max_x - min_x);
        const float z_norm = (point.z - min_z) / std::max(1e-5f, max_z - min_z);
        const int x = 12 + static_cast<int>(x_norm * (size.width - 24));
        const int y = size.height - 12 - static_cast<int>(z_norm * (size.height - 24));
        return cv::Point(x, y);
    };

    for(size_t i = 1; i < camera_positions.size(); ++i)
        cv::line(view, project(camera_positions[i - 1]), project(camera_positions[i]), cv::Scalar(80, 80, 255), 1, cv::LINE_AA);

    const cv::Point center = project(current_camera);
    cv::rectangle(view, cv::Rect(center.x - 5, center.y - 5, 10, 10), cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    return view;
}

cv::Mat MakeSceneMapView(const cv::Size& size,
                         const std::vector<monocular_slam::Point3D>& scene_points)
{
    cv::Mat view(size, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(view, "scene map", cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    if(scene_points.empty())
        return view;

    float min_x = scene_points.front().x;
    float max_x = scene_points.front().x;
    float min_y = scene_points.front().y;
    float max_y = scene_points.front().y;
    float min_z = scene_points.front().z;
    float max_z = scene_points.front().z;
    for(const monocular_slam::Point3D& point : scene_points)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    const float scale_x = std::max(1e-5f, max_x - min_x);
    const float scale_y = std::max(1e-5f, max_y - min_y);
    const float scale_z = std::max(1e-5f, max_z - min_z);

    for(size_t i = 0; i < scene_points.size(); i += 2)
    {
        const monocular_slam::Point3D& point = scene_points[i];
        const float x_norm = (point.x - min_x) / scale_x - 0.5f;
        const float y_norm = (point.y - min_y) / scale_y;
        const float z_norm = (point.z - min_z) / scale_z - 0.5f;

        const float iso_x = x_norm - 0.6f * z_norm;
        const float iso_y = (1.0f - y_norm) + 0.35f * z_norm;

        const int px = size.width / 2 + static_cast<int>(iso_x * (size.width * 0.7f));
        const int py = 24 + static_cast<int>(iso_y * (size.height - 36));
        if(px >= 0 && py >= 0 && px < size.width && py < size.height)
            cv::circle(view, cv::Point(px, py), 1, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    }

    return view;
}

std::vector<monocular_slam::Point3D> ReconstructScenePoints(const monocular_slam::FrameData& frame,
                                                            float fx,
                                                            float fy,
                                                            float cx,
                                                            float cy)
{
    std::vector<monocular_slam::Point3D> scene_points;
    if(!frame.has_pose || frame.estimated_depth.empty() || fx <= 0.0f || fy <= 0.0f)
        return scene_points;

    cv::Mat depth_float;
    if(frame.estimated_depth.type() == CV_32F)
        depth_float = frame.estimated_depth;
    else
        frame.estimated_depth.convertTo(depth_float, CV_32F);

    const cv::Matx33f Rcw = RotationFromPose(frame.pose_matrix);
    const cv::Matx33f Rwc = Rcw.t();
    const cv::Vec3f tcw = TranslationFromPose(frame.pose_matrix);

    const int stride = 10;
    scene_points.reserve(static_cast<size_t>((depth_float.rows / stride + 1) * (depth_float.cols / stride + 1)));
    for(int y = 0; y < depth_float.rows; y += stride)
    {
        for(int x = 0; x < depth_float.cols; x += stride)
        {
            const float z = depth_float.at<float>(y, x);
            if(z <= 1e-6f)
                continue;

            const float xc = (static_cast<float>(x) - cx) * z / fx;
            const float yc = (static_cast<float>(y) - cy) * z / fy;
            const cv::Vec3f Xc(xc, yc, z);
            const cv::Vec3f Xw = Rwc * (Xc - tcw);

            monocular_slam::Point3D point;
            point.x = Xw[0];
            point.y = Xw[1];
            point.z = Xw[2];
            scene_points.push_back(point);
        }
    }
    return scene_points;
}

cv::Mat ComposeDebugView(const cv::Mat& input_bgr,
                         const monocular_slam::FrameData& current,
                         const monocular_slam::FrameData& previous,
                         const std::vector<cv::Point3f>& camera_positions,
                         const std::vector<monocular_slam::Point3D>& scene_points)
{
    cv::Mat annotated = input_bgr.clone();

    const cv::Mat& display_mask = PickLatestAvailableMat(current.dynamic_mask, previous.dynamic_mask);
    const cv::Mat& display_depth = PickLatestAvailableMat(current.estimated_depth, previous.estimated_depth);
    const monocular_slam::MaskPropagationData& display_propagation =
        current.mask_propagation.attempted ? current.mask_propagation : previous.mask_propagation;

    if(!display_mask.empty())
    {
        cv::Mat overlay = annotated.clone();
        overlay.setTo(cv::Scalar(0, 0, 255), display_mask);
        cv::addWeighted(overlay, 0.22, annotated, 0.78, 0.0, annotated);
    }

    DrawDetections(annotated, current.yolo_detections);
    DrawPoints(annotated, current.filtered_orb_keypoints, cv::Scalar(0, 255, 0), 2, 1, 4);
    DrawPoints(annotated, current.tracked_keypoints, cv::Scalar(255, 255, 0), 2, -1, 4);
    DrawTextBlock(annotated, current, previous);

    const int preview_width = 280;
    const int preview_height = std::max(annotated.rows / 4, 160);
    cv::Mat depth_preview = MakeDepthPreview(display_depth);
    cv::Mat mask_preview = MakeMaskPreview(display_mask, display_propagation);
    cv::Point3f current_camera(0.0f, 0.0f, 0.0f);
    if(current.has_pose)
        current_camera = CameraWorldPositionFromPose(current.pose_matrix);
    cv::Mat pose_preview = MakeTopDownView(cv::Size(preview_width, preview_height), camera_positions, current_camera);
    cv::Mat scene_preview = MakeSceneMapView(cv::Size(preview_width, preview_height), scene_points);
    if(depth_preview.empty())
        depth_preview = MakePlaceholderPreview(cv::Size(preview_width, preview_height), "no depth");
    if(mask_preview.empty())
        mask_preview = MakePlaceholderPreview(cv::Size(preview_width, preview_height), "no dynamic mask");

    cv::resize(depth_preview, depth_preview, cv::Size(preview_width, preview_height));
    cv::resize(mask_preview, mask_preview, cv::Size(preview_width, preview_height));

    cv::putText(depth_preview, "estimated depth", cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(mask_preview, "mask + propagation", cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::Mat right_column;
    cv::Mat top_pair;
    cv::Mat bottom_pair;
    cv::vconcat(depth_preview, mask_preview, top_pair);
    cv::vconcat(pose_preview, scene_preview, bottom_pair);
    cv::vconcat(top_pair, bottom_pair, right_column);
    if(right_column.rows != annotated.rows)
        cv::resize(right_column, right_column, cv::Size(right_column.cols, annotated.rows));

    cv::Mat composed;
    cv::hconcat(annotated, right_column, composed);
    return composed;
}

} // namespace

int main(int argc, char** argv)
{
    if(argc != 4)
    {
        std::cerr << std::endl
                  << "Usage: ./dataset_debug path_to_vocabulary path_to_settings path_to_sequence" << std::endl;
        return 1;
    }

    std::vector<std::string> image_filenames;
    std::vector<double> timestamps;
    LoadTumImages(std::string(argv[3]) + "/rgb.txt", image_filenames, timestamps);

    if(image_filenames.empty())
    {
        std::cerr << "No images found in dataset." << std::endl;
        return 1;
    }

    monocular_slam::SessionConfig config;
    config.vocabulary_path = argv[1];
    config.settings_path = argv[2];
    config.use_viewer = false;
    config.sensor_mode = monocular_slam::SensorMode::Monocular;

    monocular_slam::MonocularPipeline pipeline;
    if(!pipeline.Initialize(config))
    {
        std::cerr << "Failed to initialize monocular pipeline." << std::endl;
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

    cv::namedWindow("dataset_debug", cv::WINDOW_NORMAL);
    std::vector<cv::Point3f> camera_positions;
    std::vector<monocular_slam::Point3D> accumulated_scene_points;
    std::vector<float> vTimesTrack;
    vTimesTrack.reserve(image_filenames.size());

    for(std::size_t i = 0; i < image_filenames.size(); ++i)
    {
        const std::string frame_path = std::string(argv[3]) + "/" + image_filenames[i];
        cv::Mat frame = cv::imread(frame_path, cv::IMREAD_UNCHANGED);
        if(frame.empty())
        {
            std::cerr << "Failed to load image: " << frame_path << std::endl;
            return 1;
        }

        if(frame.channels() == 1)
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        else if(frame.channels() == 4)
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);

        const auto t1 = std::chrono::steady_clock::now();
        const monocular_slam::FrameData current = pipeline.ProcessFrame(frame, timestamps[i], image_filenames[i]);
        const monocular_slam::FrameData previous = pipeline.GetPreviousFrameData();
        const auto t2 = std::chrono::steady_clock::now();

        if(current.has_pose)
            camera_positions.push_back(CameraWorldPositionFromPose(current.pose_matrix));
        const std::vector<monocular_slam::Point3D> reconstructed_points = ReconstructScenePoints(current, fx, fy, cx, cy);
        for(const monocular_slam::Point3D& point : reconstructed_points)
            accumulated_scene_points.push_back(point);
        if(camera_positions.size() > 4000)
            camera_positions.erase(camera_positions.begin(), camera_positions.begin() + static_cast<long>(camera_positions.size() - 4000));
        if(accumulated_scene_points.size() > 60000)
            accumulated_scene_points.erase(accumulated_scene_points.begin(), accumulated_scene_points.begin() + static_cast<long>(accumulated_scene_points.size() - 60000));

        cv::Mat debug_view = ComposeDebugView(frame, current, previous, camera_positions, accumulated_scene_points);
        cv::imshow("dataset_debug", debug_view);

        double wait_seconds = 0.0;
        if(i + 1 < timestamps.size())
            wait_seconds = timestamps[i + 1] - timestamps[i];

        const double track_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack.push_back(static_cast<float>(track_seconds));
        if(track_seconds < wait_seconds)
            std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds - track_seconds));

        const int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q')
            break;
    }

    cv::destroyAllWindows();

    if(!vTimesTrack.empty())
    {
        std::sort(vTimesTrack.begin(), vTimesTrack.end());
        float total_time = 0.0f;
        for(float t : vTimesTrack)
            total_time += t;
        const float median_track_time = vTimesTrack[vTimesTrack.size() / 2];
        const float mean_track_time = total_time / static_cast<float>(vTimesTrack.size());

        std::cout << "-------" << std::endl << std::endl;
        std::cout << "median tracking time: " << median_track_time << std::endl;
        std::cout << "mean tracking time: " << mean_track_time << std::endl;

        pipeline.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        pipeline.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        const std::string groundtruth_file = std::string(argv[3]) + "/groundtruth.txt";
        std::ifstream gt(groundtruth_file.c_str());
        if(gt.good())
        {
            const std::string dataset_name = Basename(argv[3]);
            const std::string metrics_csv = "out/dataset_debug_metrics.csv";
            std::ostringstream cmd;
            cmd << "python3 evaluation/evaluate_tum_metrics.py "
                << "\"" << groundtruth_file << "\" "
                << "\"CameraTrajectory.txt\" "
                << "--tracking-time-sec "
                << std::fixed << std::setprecision(9) << mean_track_time << " "
                << "--median-tracking-time-sec "
                << std::fixed << std::setprecision(9) << median_track_time << " "
                << "--dataset-name "
                << "\"" << dataset_name << "\" "
                << "--csv "
                << "\"" << metrics_csv << "\"";

            std::cout << std::endl << "Evaluating trajectory metrics..." << std::endl;
            const int eval_status = std::system(cmd.str().c_str());
            if(eval_status != 0)
                std::cerr << "Trajectory metric evaluation failed with exit code " << eval_status << std::endl;
        }
    }

    std::cout.flush();
    std::cerr.flush();
    std::_Exit(0);
}
