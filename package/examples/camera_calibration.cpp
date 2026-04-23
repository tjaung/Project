#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "monocular_slam/Calibration.h"

namespace
{

bool IsUnsignedInteger(const std::string& value)
{
    if(value.empty())
        return false;
    for(char c : value)
    {
        if(!std::isdigit(static_cast<unsigned char>(c)))
            return false;
    }
    return true;
}

std::string NormalizeVideoSource(const std::string& source)
{
    if(source.rfind("http://", 0) == 0 || source.rfind("https://", 0) == 0 || source.rfind("rtsp://", 0) == 0)
        return source;

    if(source.find('/') != std::string::npos)
        return source;

    const bool looks_like_host =
        source.find('.') != std::string::npos || source.find(':') != std::string::npos || source == "localhost";
    if(looks_like_host)
        return "http://" + source + "/video";

    return source;
}

bool EnsureParentDirectory(const std::string& path)
{
    const std::size_t slash = path.find_last_of("/\\");
    if(slash == std::string::npos)
        return true;

    const std::string dir = path.substr(0, slash);
    if(dir.empty())
        return true;

    std::string partial;
    for(std::size_t i = 0; i < dir.size(); ++i)
    {
        partial.push_back(dir[i]);
        if(dir[i] != '/')
            continue;
        if(partial.size() == 1)
            continue;
        mkdir(partial.c_str(), 0755);
    }
    mkdir(dir.c_str(), 0755);
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if(argc < 5 || argc > 7)
    {
        std::cerr << "Usage: ./camera_calibration cols rows square_size_m camera_index_video_path_or_ipcam [output_yaml] [required_frames]\n"
                  << "Examples:\n"
                  << "  ./examples/camera_calibration 9 6 0.024 0\n"
                  << "  ./examples/camera_calibration 9 6 0.024 http://192.168.1.20:8080/video out/calibration/phone_calibrated.yaml 20\n"
                  << "  ./examples/camera_calibration 9 6 0.024 192.168.1.20:8080 out/calibration/phone_calibrated.yaml\n";
        return 1;
    }

    monocular_slam::ChessboardCalibrationConfig config;
    config.board_size = cv::Size(std::stoi(argv[1]), std::stoi(argv[2]));
    config.square_size_m = std::stof(argv[3]);

    const std::string source = argv[4];
    const std::string default_output_path = "out/calibration/webcam_calibrated.yaml";
    std::string output_path = default_output_path;
    if(argc == 6)
    {
        if(IsUnsignedInteger(argv[5]))
            config.required_frames = std::max(5, std::stoi(argv[5]));
        else
            output_path = argv[5];
    }
    else if(argc == 7)
    {
        output_path = argv[5];
        config.required_frames = std::max(5, std::stoi(argv[6]));
    }

    const std::string normalized_source = NormalizeVideoSource(source);

    cv::VideoCapture cap;
    if(IsUnsignedInteger(source))
        cap.open(std::stoi(source));
    else
        cap.open(normalized_source);

    if(!cap.isOpened())
    {
        std::cerr << "Failed to open calibration source: " << source << "\n";
        if(normalized_source != source)
            std::cerr << "Tried normalized IP camera URL: " << normalized_source << "\n";
        return 1;
    }

    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    double source_fps = cap.get(cv::CAP_PROP_FPS);
    if(!std::isfinite(source_fps) || source_fps < 5.0 || source_fps > 120.0)
        source_fps = 15.0;

    std::cout << "Opened calibration source: "
              << (IsUnsignedInteger(source) ? ("camera index " + source) : normalized_source)
              << "\n";
    std::cout << "Using live-camera settings FPS hint: " << source_fps << "\n";

    std::vector<std::vector<cv::Point2f>> all_corners;
    cv::Size image_size;

    std::cout << "Controls:\n";
    std::cout << "  Space: capture current chessboard if detected\n";
    std::cout << "  Enter: finish and calibrate once enough captures exist\n";
    std::cout << "  Esc: quit without saving\n";
    std::cout << "Output YAML: " << output_path << "\n";

    while(true)
    {
        cv::Mat frame;
        if(!cap.read(frame) || frame.empty())
            break;

        image_size = frame.size();

        std::vector<cv::Point2f> corners;
        cv::Mat debug;
        const bool found = monocular_slam::FindChessboardCorners(frame, config, corners, &debug);

        const std::string status = found ? "Chessboard detected" : "No chessboard";
        cv::putText(debug,
                    status,
                    cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    found ? cv::Scalar(50, 220, 50) : cv::Scalar(40, 40, 230),
                    2,
                    cv::LINE_AA);
        cv::putText(debug,
                    "Captured: " + std::to_string(all_corners.size()) + "/" + std::to_string(config.required_frames),
                    cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 230, 90),
                    2,
                    cv::LINE_AA);

        cv::imshow("camera_calibration", debug);
        const int key = cv::waitKey(1);

        if(key == 27)
            return 0;

        if((key == ' ' || key == 'c' || key == 'C') && found)
        {
            all_corners.push_back(corners);
            std::cout << "Captured frame " << all_corners.size() << " / " << config.required_frames << std::endl;
        }

        if((key == 13 || key == 10) && static_cast<int>(all_corners.size()) >= config.required_frames)
            break;
    }

    monocular_slam::ChessboardCalibrationResult result =
        monocular_slam::CalibrateCamera(all_corners, image_size, config);

    if(!result.success)
    {
        std::cerr << "Calibration failed.\n";
        return 1;
    }

    EnsureParentDirectory(output_path);
    if(!monocular_slam::WriteOrbSlamMonocularSettingsFile(output_path, result, source_fps, true))
    {
        std::cerr << "Failed to write calibration YAML to " << output_path << std::endl;
        return 1;
    }

    std::cout << "Calibration successful.\n";
    std::cout << "RMS reprojection error: " << result.rms_reprojection_error << "\n";
    std::cout << "Saved settings to: " << output_path << "\n";
    return 0;
}
