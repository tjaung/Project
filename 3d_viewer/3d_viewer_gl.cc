#define GL_SILENCE_DEPRECATION

#include <GLFW/glfw3.h>
#include <OpenGL/glu.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Mat3
{
    float m[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
};

struct ColoredPoint
{
    Vec3 position;
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
};

struct ViewerState
{
    Vec3 cameraPosition;
    Mat3 cameraRotation;
    std::vector<Vec3> trajectory;
    std::vector<ColoredPoint> points;
};

struct OrbitCamera
{
    float yaw = 0.8f;
    float pitch = -0.45f;
    float distance = 6.0f;
    Vec3 target {0.0f, 0.0f, 0.0f};
    bool follow = false;
    bool showPoints = true;
    bool showTrajectory = true;
    bool showCamera = true;
};

struct AppState
{
    std::string snapshotPath;
    ViewerState viewerState;
    bool hasState = false;
    bool hasAutoFramed = false;
    OrbitCamera orbit;
    bool leftDragging = false;
    bool rightDragging = false;
    double lastX = 0.0;
    double lastY = 0.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }
Vec3 operator/(const Vec3& v, float s) { return {v.x / s, v.y / s, v.z / s}; }

float Dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 Cross(const Vec3& a, const Vec3& b) { return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}; }
float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }
Vec3 Normalize(const Vec3& v) { const float len = Length(v); return len < 1e-6f ? Vec3{} : v * (1.0f / len); }

bool LoadState(const std::string& path, ViewerState& state)
{
    std::ifstream in(path.c_str());
    if(!in.is_open())
        return false;

    std::string magic;
    std::getline(in, magic);
    if(magic != "MDSLAM_LIVE_CLOUD_V1")
        return false;

    std::string line;
    if(!std::getline(in, line))
        return false;
    {
        std::istringstream cameraLine(line);
        std::string section;
        cameraLine >> section;
        if(section != "CAMERA")
            return false;
        std::vector<float> values;
        float value = 0.0f;
        while(cameraLine >> value)
            values.push_back(value);
        if(values.size() != 12)
            return false;

        state.cameraPosition = {values[0], values[1], values[2]};
        state.cameraRotation = Mat3{};
        state.cameraRotation.m[0] = values[3];
        state.cameraRotation.m[1] = values[4];
        state.cameraRotation.m[2] = values[5];
        state.cameraRotation.m[3] = values[6];
        state.cameraRotation.m[4] = values[7];
        state.cameraRotation.m[5] = values[8];
        state.cameraRotation.m[6] = values[9];
        state.cameraRotation.m[7] = values[10];
        state.cameraRotation.m[8] = values[11];
    }

    if(!std::getline(in, line))
        return false;
    size_t count = 0;
    {
        std::istringstream header(line);
        std::string section;
        header >> section >> count;
        if(section != "TRAJECTORY")
            return false;
    }
    state.trajectory.clear();
    state.trajectory.reserve(count);
    for(size_t i = 0; i < count; ++i)
    {
        if(!std::getline(in, line))
            return false;
        Vec3 p;
        std::istringstream s(line);
        s >> p.x >> p.y >> p.z;
        if(!s)
            return false;
        state.trajectory.push_back(p);
    }

    if(!std::getline(in, line))
        return false;
    {
        std::istringstream header(line);
        std::string section;
        header >> section >> count;
        if(section != "POINTS")
            return false;
    }
    state.points.clear();
    state.points.reserve(count);
    for(size_t i = 0; i < count; ++i)
    {
        if(!std::getline(in, line))
            return false;
        ColoredPoint point;
        int r = 255, g = 255, b = 255;
        std::istringstream s(line);
        s >> point.position.x >> point.position.y >> point.position.z >> r >> g >> b;
        if(!s)
            return false;
        point.r = static_cast<float>(r) / 255.0f;
        point.g = static_cast<float>(g) / 255.0f;
        point.b = static_cast<float>(b) / 255.0f;
        state.points.push_back(point);
    }

    return true;
}

void DrawGrid(float halfSpan = 10.0f, float step = 1.0f)
{
    glColor3f(0.88f, 0.88f, 0.88f);
    glBegin(GL_LINES);
    for(float x = -halfSpan; x <= halfSpan; x += step)
    {
        glVertex3f(x, 0.0f, -halfSpan);
        glVertex3f(x, 0.0f, halfSpan);
    }
    for(float z = -halfSpan; z <= halfSpan; z += step)
    {
        glVertex3f(-halfSpan, 0.0f, z);
        glVertex3f(halfSpan, 0.0f, z);
    }
    glEnd();
}

void DrawAxes(float scale = 1.0f)
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(0.9f, 0.2f, 0.2f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(scale, 0.0f, 0.0f);
    glColor3f(0.2f, 0.8f, 0.2f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, scale, 0.0f);
    glColor3f(0.2f, 0.4f, 0.9f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, scale);
    glEnd();
    glLineWidth(1.0f);
}

void DrawPoints(const ViewerState& state)
{
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for(const ColoredPoint& point : state.points)
    {
        glColor3f(point.r, point.g, point.b);
        glVertex3f(point.position.x, point.position.y, point.position.z);
    }
    glEnd();
}

void DrawTrajectory(const ViewerState& state)
{
    if(state.trajectory.size() < 2)
        return;
    glLineWidth(2.0f);
    glColor3f(1.0f, 0.5f, 0.0f);
    glBegin(GL_LINE_STRIP);
    for(const Vec3& p : state.trajectory)
        glVertex3f(p.x, p.y, p.z);
    glEnd();
    glLineWidth(1.0f);
}

void ApplyPose(const Vec3& position, const Mat3& rotation)
{
    GLfloat matrix[16] = {
        rotation.m[0], rotation.m[3], rotation.m[6], 0.0f,
        rotation.m[1], rotation.m[4], rotation.m[7], 0.0f,
        rotation.m[2], rotation.m[5], rotation.m[8], 0.0f,
        position.x,     position.y,     position.z,     1.0f
    };
    glMultMatrixf(matrix);
}

void DrawFrustum(const Vec3& position, const Mat3& rotation, float scale)
{
    const float w = scale;
    const float h = scale * 0.75f;
    const float z = scale * 0.6f;

    glPushMatrix();
    ApplyPose(position, rotation);
    glLineWidth(2.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0); glVertex3f(w,h,z);
    glVertex3f(0,0,0); glVertex3f(w,-h,z);
    glVertex3f(0,0,0); glVertex3f(-w,-h,z);
    glVertex3f(0,0,0); glVertex3f(-w,h,z);
    glVertex3f(w,h,z); glVertex3f(w,-h,z);
    glVertex3f(-w,h,z); glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z); glVertex3f(w,h,z);
    glVertex3f(-w,-h,z); glVertex3f(w,-h,z);
    glEnd();
    glLineWidth(1.0f);
    glPopMatrix();
}

void SetPerspective(int width, int height)
{
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, aspect, 0.01, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

Vec3 OrbitEye(const OrbitCamera& orbit)
{
    const float cp = std::cos(orbit.pitch);
    return {
        orbit.target.x + orbit.distance * cp * std::sin(orbit.yaw),
        orbit.target.y + orbit.distance * std::sin(orbit.pitch),
        orbit.target.z + orbit.distance * cp * std::cos(orbit.yaw)
    };
}

bool ComputeSceneBounds(const ViewerState& state, Vec3& minCorner, Vec3& maxCorner)
{
    bool hasAny = false;
    minCorner = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    maxCorner = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
    auto extend = [&](const Vec3& p)
    {
        hasAny = true;
        minCorner.x = std::min(minCorner.x, p.x);
        minCorner.y = std::min(minCorner.y, p.y);
        minCorner.z = std::min(minCorner.z, p.z);
        maxCorner.x = std::max(maxCorner.x, p.x);
        maxCorner.y = std::max(maxCorner.y, p.y);
        maxCorner.z = std::max(maxCorner.z, p.z);
    };
    for(const ColoredPoint& point : state.points)
        extend(point.position);
    for(const Vec3& p : state.trajectory)
        extend(p);
    extend(state.cameraPosition);
    return hasAny;
}

void AutoFrameOrbit(AppState& app)
{
    if(!app.hasState)
        return;
    Vec3 minCorner, maxCorner;
    if(!ComputeSceneBounds(app.viewerState, minCorner, maxCorner))
        return;
    const Vec3 center = (minCorner + maxCorner) / 2.0f;
    const Vec3 diagonal = maxCorner - minCorner;
    const float radius = std::max(Length(diagonal) * 0.7f, 0.8f);
    app.orbit.target = center;
    app.orbit.distance = std::min(std::max(radius * 2.0f, 2.5f), 120.0f);
    app.orbit.yaw = 0.8f;
    app.orbit.pitch = -0.45f;
}

void UpdateFollowCamera(AppState& app)
{
    if(app.orbit.follow && app.hasState)
        app.orbit.target = app.viewerState.cameraPosition;
}

void SetPresetView(OrbitCamera& orbit, int preset)
{
    orbit.follow = false;
    if(preset == 1) { orbit.yaw = 0.8f; orbit.pitch = -0.45f; orbit.distance = 6.0f; }
    else if(preset == 2) { orbit.yaw = 0.0f; orbit.pitch = -1.45f; orbit.distance = 12.0f; }
    else if(preset == 3) { orbit.yaw = 0.0f; orbit.pitch = 0.0f; orbit.distance = 8.0f; }
}

time_t GetFileModificationTime(const std::string& path)
{
    if(path.empty())
        return 0;
    struct stat info;
    if(stat(path.c_str(), &info) != 0)
        return 0;
    return info.st_mtime;
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int)
{
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if(!app)
        return;
    if(button == GLFW_MOUSE_BUTTON_LEFT) app->leftDragging = action == GLFW_PRESS;
    if(button == GLFW_MOUSE_BUTTON_RIGHT) app->rightDragging = action == GLFW_PRESS;
    glfwGetCursorPos(window, &app->lastX, &app->lastY);
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if(!app)
        return;
    const double dx = xpos - app->lastX;
    const double dy = ypos - app->lastY;
    app->lastX = xpos;
    app->lastY = ypos;
    if(app->leftDragging)
    {
        app->orbit.follow = false;
        app->orbit.yaw += static_cast<float>(dx) * 0.01f;
        app->orbit.pitch += static_cast<float>(dy) * 0.01f;
        app->orbit.pitch = std::max(-1.5f, std::min(1.5f, app->orbit.pitch));
    }
    if(app->rightDragging)
    {
        app->orbit.follow = false;
        const Vec3 eye = OrbitEye(app->orbit);
        const Vec3 forward = Normalize(app->orbit.target - eye);
        const Vec3 worldUp {0.0f, 1.0f, 0.0f};
        const Vec3 right = Normalize(Cross(forward, worldUp));
        const Vec3 up = Normalize(Cross(right, forward));
        const float panScale = 0.0025f * app->orbit.distance;
        app->orbit.target = app->orbit.target - right * static_cast<float>(dx) * panScale + up * static_cast<float>(dy) * panScale;
    }
}

void ScrollCallback(GLFWwindow* window, double, double yoffset)
{
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if(!app)
        return;
    app->orbit.follow = false;
    app->orbit.distance *= (yoffset > 0.0) ? 0.9f : 1.1f;
    app->orbit.distance = std::max(0.1f, std::min(200.0f, app->orbit.distance));
}

void KeyCallback(GLFWwindow* window, int key, int, int action, int)
{
    if(action != GLFW_PRESS)
        return;
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if(!app)
        return;
    switch(key)
    {
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
    case GLFW_KEY_A: AutoFrameOrbit(*app); break;
    case GLFW_KEY_F: app->orbit.follow = !app->orbit.follow; break;
    case GLFW_KEY_P: app->orbit.showPoints = !app->orbit.showPoints; break;
    case GLFW_KEY_T: app->orbit.showTrajectory = !app->orbit.showTrajectory; break;
    case GLFW_KEY_C: app->orbit.showCamera = !app->orbit.showCamera; break;
    case GLFW_KEY_1: SetPresetView(app->orbit, 1); break;
    case GLFW_KEY_2: SetPresetView(app->orbit, 2); break;
    case GLFW_KEY_3: SetPresetView(app->orbit, 3); break;
    default: break;
    }
}

void RenderScene(AppState& app, int width, int height)
{
    glViewport(0, 0, width, height);
    glClearColor(0.97f, 0.97f, 0.99f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    SetPerspective(width, height);
    glLoadIdentity();

    UpdateFollowCamera(app);
    const Vec3 eye = OrbitEye(app.orbit);
    gluLookAt(eye.x, eye.y, eye.z,
              app.orbit.target.x, app.orbit.target.y, app.orbit.target.z,
              0.0f, 1.0f, 0.0f);

    DrawGrid();
    DrawAxes();
    if(app.hasState)
    {
        if(app.orbit.showPoints)
            DrawPoints(app.viewerState);
        if(app.orbit.showTrajectory)
            DrawTrajectory(app.viewerState);
        if(app.orbit.showCamera)
            DrawFrustum(app.viewerState.cameraPosition, app.viewerState.cameraRotation, 0.14f);
    }
}

void UpdateWindowTitle(GLFWwindow* window, const AppState& app)
{
    std::ostringstream title;
    title << "3d_viewer";
    if(app.hasState)
        title << " | points=" << app.viewerState.points.size() << " traj=" << app.viewerState.trajectory.size();
    glfwSetWindowTitle(window, title.str().c_str());
}

} // namespace

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << "Usage: ./3d_viewer_gl path_to_live_cloud_snapshot.txt" << std::endl;
        return 1;
    }

    if(!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow* window = glfwCreateWindow(1280, 800, "3d_viewer", nullptr, nullptr);
    if(!window)
    {
        std::cerr << "Failed to create window." << std::endl;
        glfwTerminate();
        return 1;
    }

    AppState app;
    app.snapshotPath = argv[1];
    glfwSetWindowUserPointer(window, &app);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetCursorPosCallback(window, CursorPosCallback);
    glfwSetScrollCallback(window, ScrollCallback);
    glfwSetKeyCallback(window, KeyCallback);

    time_t lastSnapshotTime = 0;
    while(!glfwWindowShouldClose(window))
    {
        const time_t snapshotTime = GetFileModificationTime(app.snapshotPath);
        if(snapshotTime != 0 && snapshotTime != lastSnapshotTime)
        {
            ViewerState newState;
            if(LoadState(app.snapshotPath, newState))
            {
                app.viewerState = newState;
                app.hasState = true;
                lastSnapshotTime = snapshotTime;
                if(!app.hasAutoFramed)
                {
                    AutoFrameOrbit(app);
                    app.hasAutoFramed = true;
                }
                UpdateWindowTitle(window, app);
            }
        }

        int width = 0, height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        RenderScene(app, width, height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
