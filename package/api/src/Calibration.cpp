#include "monocular_slam/Calibration.h"

#include <cmath>
#include <fstream>
#include <iomanip>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace monocular_slam
{

namespace
{

std::vector<cv::Point3f> BuildObjectPoints(const ChessboardCalibrationConfig& config)
{
    std::vector<cv::Point3f> object_points;
    object_points.reserve(static_cast<std::size_t>(config.board_size.width * config.board_size.height));
    for(int row = 0; row < config.board_size.height; ++row)
    {
        for(int col = 0; col < config.board_size.width; ++col)
            object_points.emplace_back(col * config.square_size_m, row * config.square_size_m, 0.0f);
    }
    return object_points;
}

} // namespace

bool FindChessboardCorners(const cv::Mat& image,
                           const ChessboardCalibrationConfig& config,
                           std::vector<cv::Point2f>& corners,
                           cv::Mat* debug_image)
{
    corners.clear();
    if(image.empty())
        return false;

    cv::Mat gray;
    if(image.channels() == 1)
        gray = image;
    else
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat normalized;
    cv::equalizeHist(gray, normalized);

    const int classic_flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool found = cv::findChessboardCorners(normalized, config.board_size, corners, classic_flags);

    if(!found)
    {
        const int sb_flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
        found = cv::findChessboardCornersSB(normalized, config.board_size, corners, sb_flags);
    }

    if(found && config.refine_corners)
    {
        cv::cornerSubPix(normalized,
                         corners,
                         cv::Size(11, 11),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 50, 0.0005));
    }

    if(debug_image)
    {
        if(image.channels() == 3)
            *debug_image = image.clone();
        else
            cv::cvtColor(image, *debug_image, cv::COLOR_GRAY2BGR);
        cv::drawChessboardCorners(*debug_image, config.board_size, corners, found);
    }

    return found;
}

ChessboardCalibrationResult CalibrateCamera(
    const std::vector<std::vector<cv::Point2f>>& all_corners,
    const cv::Size& image_size,
    const ChessboardCalibrationConfig& config)
{
    ChessboardCalibrationResult result;
    result.image_size = image_size;
    result.used_frame_count = static_cast<int>(all_corners.size());

    if(all_corners.empty() || image_size.width <= 0 || image_size.height <= 0)
        return result;

    std::vector<std::vector<cv::Point3f>> object_points(all_corners.size(), BuildObjectPoints(config));
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    result.distortion_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    result.rms_reprojection_error = cv::calibrateCamera(object_points,
                                                        all_corners,
                                                        image_size,
                                                        result.camera_matrix,
                                                        result.distortion_coeffs,
                                                        rvecs,
                                                        tvecs);

    result.success = cv::checkRange(result.camera_matrix) && cv::checkRange(result.distortion_coeffs);
    return result;
}

bool WriteOrbSlamMonocularSettingsFile(
    const std::string& path,
    const ChessboardCalibrationResult& calibration,
    double fps,
    bool rgb)
{
    if(!calibration.success || calibration.camera_matrix.empty() || calibration.distortion_coeffs.empty())
        return false;

    const double fx = calibration.camera_matrix.at<double>(0, 0);
    const double fy = calibration.camera_matrix.at<double>(1, 1);
    const double cx = calibration.camera_matrix.at<double>(0, 2);
    const double cy = calibration.camera_matrix.at<double>(1, 2);

    const double k1 = calibration.distortion_coeffs.total() > 0 ? calibration.distortion_coeffs.at<double>(0, 0) : 0.0;
    const double k2 = calibration.distortion_coeffs.total() > 1 ? calibration.distortion_coeffs.at<double>(1, 0) : 0.0;
    const double p1 = calibration.distortion_coeffs.total() > 2 ? calibration.distortion_coeffs.at<double>(2, 0) : 0.0;
    const double p2 = calibration.distortion_coeffs.total() > 3 ? calibration.distortion_coeffs.at<double>(3, 0) : 0.0;

    std::ofstream out(path.c_str());
    if(!out.is_open())
        return false;

    out << "%YAML:1.0\n\n";
    out << "File.version: \"1.0\"\n\n";
    out << "Camera.type: \"PinHole\"\n\n";
    out << std::fixed << std::setprecision(8);
    out << "Camera1.fx: " << fx << "\n";
    out << "Camera1.fy: " << fy << "\n";
    out << "Camera1.cx: " << cx << "\n";
    out << "Camera1.cy: " << cy << "\n\n";
    out << "Camera1.k1: " << k1 << "\n";
    out << "Camera1.k2: " << k2 << "\n";
    out << "Camera1.p1: " << p1 << "\n";
    out << "Camera1.p2: " << p2 << "\n\n";
    out << std::setprecision(0);
    out << "Camera.fps: " << std::llround(fps) << "\n";
    out << std::setprecision(8);
    out << "Camera.RGB: " << (rgb ? 1 : 0) << "\n";
    out << "Camera.width: " << calibration.image_size.width << "\n";
    out << "Camera.height: " << calibration.image_size.height << "\n\n";
    out << "ORBextractor.nFeatures: 1800\n";
    out << "ORBextractor.scaleFactor: 1.2\n";
    out << "ORBextractor.nLevels: 8\n";
    out << "ORBextractor.iniThFAST: 15\n";
    out << "ORBextractor.minThFAST: 5\n\n";
    out << "Viewer.KeyFrameSize: 0.05\n";
    out << "Viewer.KeyFrameLineWidth: 1.0\n";
    out << "Viewer.GraphLineWidth: 0.9\n";
    out << "Viewer.PointSize: 2.0\n";
    out << "Viewer.CameraSize: 0.08\n";
    out << "Viewer.CameraLineWidth: 3.0\n";
    out << "Viewer.ViewpointX: 0.0\n";
    out << "Viewer.ViewpointY: -0.7\n";
    out << "Viewer.ViewpointZ: -1.8\n";
    out << "Viewer.ViewpointF: 500.0\n";
    return true;
}

} // namespace monocular_slam
