#ifndef MONOCULAR_SLAM_CALIBRATION_H
#define MONOCULAR_SLAM_CALIBRATION_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace monocular_slam
{

struct ChessboardCalibrationConfig
{
    cv::Size board_size = cv::Size(9, 6);
    float square_size_m = 0.024f;
    int required_frames = 20;
    bool refine_corners = true;
};

struct ChessboardCalibrationResult
{
    bool success = false;
    cv::Size image_size;
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    double rms_reprojection_error = 0.0;
    int used_frame_count = 0;
};

bool FindChessboardCorners(const cv::Mat& image,
                           const ChessboardCalibrationConfig& config,
                           std::vector<cv::Point2f>& corners,
                           cv::Mat* debug_image = nullptr);

ChessboardCalibrationResult CalibrateCamera(
    const std::vector<std::vector<cv::Point2f>>& all_corners,
    const cv::Size& image_size,
    const ChessboardCalibrationConfig& config);

bool WriteOrbSlamMonocularSettingsFile(
    const std::string& path,
    const ChessboardCalibrationResult& calibration,
    double fps = 30.0,
    bool rgb = true);

} // namespace monocular_slam

#endif
