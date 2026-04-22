#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct PointSample
{
    cv::Point3f position;
    bool isReference = false;
};

struct KeyframeSample
{
    long id = -1;
    cv::Point3f position;
};

struct EdgeSample
{
    cv::Point3f a;
    cv::Point3f b;
    int kind = 0;
};

struct CameraSample
{
    cv::Point3f position;
    cv::Point3f forward;
};

struct ViewerState
{
    CameraSample camera;
    std::vector<PointSample> points;
    std::vector<KeyframeSample> keyframes;
    std::vector<EdgeSample> edges;
};

bool LoadState(const std::string& path, ViewerState& state)
{
    std::ifstream in(path.c_str());
    if (!in.is_open())
        return false;

    std::string magic;
    in >> magic;
    if (magic != "ORB_SLAM3_DEBUG_MAP_V1")
        return false;

    std::string section;
    in >> section;
    if (section != "CAMERA")
        return false;

    in >> state.camera.position.x >> state.camera.position.y >> state.camera.position.z
       >> state.camera.forward.x >> state.camera.forward.y >> state.camera.forward.z;

    size_t count = 0;
    in >> section >> count;
    if (section != "POINTS")
        return false;
    state.points.clear();
    state.points.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        PointSample point;
        int ref = 0;
        in >> point.position.x >> point.position.y >> point.position.z >> ref;
        point.isReference = ref != 0;
        state.points.push_back(point);
    }

    in >> section >> count;
    if (section != "KEYFRAMES")
        return false;
    state.keyframes.clear();
    state.keyframes.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        KeyframeSample keyframe;
        in >> keyframe.id >> keyframe.position.x >> keyframe.position.y >> keyframe.position.z;
        state.keyframes.push_back(keyframe);
    }

    in >> section >> count;
    if (section != "EDGES")
        return false;
    state.edges.clear();
    state.edges.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        EdgeSample edge;
        in >> edge.a.x >> edge.a.y >> edge.a.z
           >> edge.b.x >> edge.b.y >> edge.b.z
           >> edge.kind;
        state.edges.push_back(edge);
    }

    return true;
}

struct Bounds
{
    float minA = 0.0f;
    float maxA = 1.0f;
    float minB = 0.0f;
    float maxB = 1.0f;
};

Bounds ComputeBounds(const ViewerState& state, bool useXZ)
{
    Bounds bounds;
    bool initialized = false;

    auto extend = [&](const cv::Point3f& p) {
        const float a = useXZ ? p.x : p.x;
        const float b = useXZ ? p.z : p.y;
        if (!initialized)
        {
            bounds.minA = bounds.maxA = a;
            bounds.minB = bounds.maxB = b;
            initialized = true;
            return;
        }

        bounds.minA = std::min(bounds.minA, a);
        bounds.maxA = std::max(bounds.maxA, a);
        bounds.minB = std::min(bounds.minB, b);
        bounds.maxB = std::max(bounds.maxB, b);
    };

    for (const PointSample& point : state.points)
        extend(point.position);
    for (const KeyframeSample& keyframe : state.keyframes)
        extend(keyframe.position);
    extend(state.camera.position);

    if (!initialized)
        return bounds;

    const float spanA = std::max(bounds.maxA - bounds.minA, 1.0f);
    const float spanB = std::max(bounds.maxB - bounds.minB, 1.0f);
    const float marginA = spanA * 0.1f;
    const float marginB = spanB * 0.1f;
    bounds.minA -= marginA;
    bounds.maxA += marginA;
    bounds.minB -= marginB;
    bounds.maxB += marginB;
    return bounds;
}

cv::Point2i ProjectPoint(const cv::Point3f& point, const Bounds& bounds, const cv::Rect& viewport, bool useXZ)
{
    const float a = useXZ ? point.x : point.x;
    const float b = useXZ ? point.z : point.y;
    const float spanA = std::max(bounds.maxA - bounds.minA, 1.0f);
    const float spanB = std::max(bounds.maxB - bounds.minB, 1.0f);
    const float u = (a - bounds.minA) / spanA;
    const float v = (b - bounds.minB) / spanB;

    const int x = viewport.x + static_cast<int>(u * viewport.width);
    const int y = viewport.y + viewport.height - static_cast<int>(v * viewport.height);
    return cv::Point2i(x, y);
}

void DrawArrow(cv::Mat& canvas, const cv::Point2i& origin, const cv::Point2i& tip, const cv::Scalar& color)
{
    cv::arrowedLine(canvas, origin, tip, color, 2, cv::LINE_AA, 0, 0.25);
}

void DrawView(cv::Mat& canvas,
              const ViewerState& state,
              const cv::Rect& viewport,
              const std::string& title,
              bool useXZ)
{
    cv::rectangle(canvas, viewport, cv::Scalar(235, 235, 235), cv::FILLED);
    cv::rectangle(canvas, viewport, cv::Scalar(190, 190, 190), 1);
    cv::putText(canvas, title, cv::Point(viewport.x + 12, viewport.y + 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(30, 30, 30), 2, cv::LINE_AA);

    const Bounds bounds = ComputeBounds(state, useXZ);

    for (const EdgeSample& edge : state.edges)
    {
        const cv::Point2i a = ProjectPoint(edge.a, bounds, viewport, useXZ);
        const cv::Point2i b = ProjectPoint(edge.b, bounds, viewport, useXZ);
        const cv::Scalar color = edge.kind == 0 ? cv::Scalar(180, 180, 180) : cv::Scalar(80, 80, 220);
        cv::line(canvas, a, b, color, 1, cv::LINE_AA);
    }

    for (const PointSample& point : state.points)
    {
        const cv::Point2i p = ProjectPoint(point.position, bounds, viewport, useXZ);
        const cv::Scalar color = point.isReference ? cv::Scalar(60, 80, 220) : cv::Scalar(40, 40, 40);
        cv::circle(canvas, p, point.isReference ? 2 : 1, color, cv::FILLED, cv::LINE_AA);
    }

    for (const KeyframeSample& keyframe : state.keyframes)
    {
        const cv::Point2i p = ProjectPoint(keyframe.position, bounds, viewport, useXZ);
        cv::circle(canvas, p, 4, cv::Scalar(220, 120, 40), 1, cv::LINE_AA);
    }

    const cv::Point2i cam = ProjectPoint(state.camera.position, bounds, viewport, useXZ);
    cv::Point3f tip3d = state.camera.position;
    if (useXZ)
        tip3d += cv::Point3f(state.camera.forward.x, 0.0f, state.camera.forward.z) * 0.5f;
    else
        tip3d += cv::Point3f(state.camera.forward.x, state.camera.forward.y, 0.0f) * 0.5f;
    const cv::Point2i tip = ProjectPoint(tip3d, bounds, viewport, useXZ);
    cv::circle(canvas, cam, 5, cv::Scalar(40, 170, 40), cv::FILLED, cv::LINE_AA);
    DrawArrow(canvas, cam, tip, cv::Scalar(40, 170, 40));
}

} // namespace

int main(int argc, char** argv)
{
    const std::string snapshotPath = argc > 1 ? argv[1] : "debug_map_state.txt";
    ViewerState state;
    bool hasState = false;

    cv::namedWindow("ORB-SLAM3 Debug Map Viewer", cv::WINDOW_AUTOSIZE);

    while (true)
    {
        ViewerState nextState;
        if (LoadState(snapshotPath, nextState))
        {
            state = nextState;
            hasState = true;
        }

        cv::Mat canvas(720, 1280, CV_8UC3, cv::Scalar(248, 248, 248));
        if (!hasState)
        {
            cv::putText(canvas, "Waiting for snapshot: " + snapshotPath,
                        cv::Point(40, 80), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
            cv::putText(canvas, "Set ORB_SLAM3_DEBUG_MAP_FILE before launching ORB-SLAM3.",
                        cv::Point(40, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(90, 90, 90), 2, cv::LINE_AA);
        }
        else
        {
            DrawView(canvas, state, cv::Rect(20, 20, 600, 680), "Top View (X/Z)", true);
            DrawView(canvas, state, cv::Rect(660, 20, 600, 680), "Side View (X/Y)", false);

            cv::putText(canvas,
                        "Keys: q/esc to quit",
                        cv::Point(930, 690),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(80, 80, 80),
                        1,
                        cv::LINE_AA);
        }

        cv::imshow("ORB-SLAM3 Debug Map Viewer", canvas);
        const int key = cv::waitKey(100);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    return 0;
}
