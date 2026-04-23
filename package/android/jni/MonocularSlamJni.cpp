#include <jni.h>

#include <array>
#include <string>

#include <opencv2/imgproc.hpp>

#include "monocular_slam/c_api.h"

namespace
{

constexpr const char* kNativeClassName = "com/timjaung/monocularslam/MonocularSlamNative";
constexpr const char* kSnapshotClassName = "com/timjaung/monocularslam/MdslamSnapshot";

std::string JStringToStdString(JNIEnv* env, jstring value)
{
    if(!env || !value)
        return std::string();

    const char* chars = env->GetStringUTFChars(value, nullptr);
    if(!chars)
        return std::string();

    std::string out(chars);
    env->ReleaseStringUTFChars(value, chars);
    return out;
}

jstring StdStringToJString(JNIEnv* env, const char* value)
{
    return env->NewStringUTF(value ? value : "");
}

bool FillSnapshotObject(JNIEnv* env, jobject snapshotObject, const mdslam_snapshot& snapshot)
{
    if(!env || !snapshotObject)
        return false;

    jclass cls = env->GetObjectClass(snapshotObject);
    if(!cls)
        return false;

    auto setBoolean = [&](const char* name, jboolean value)
    {
        jfieldID field = env->GetFieldID(cls, name, "Z");
        if(field) env->SetBooleanField(snapshotObject, field, value);
    };

    auto setInt = [&](const char* name, jint value)
    {
        jfieldID field = env->GetFieldID(cls, name, "I");
        if(field) env->SetIntField(snapshotObject, field, value);
    };

    auto setLong = [&](const char* name, jlong value)
    {
        jfieldID field = env->GetFieldID(cls, name, "J");
        if(field) env->SetLongField(snapshotObject, field, value);
    };

    auto setDouble = [&](const char* name, jdouble value)
    {
        jfieldID field = env->GetFieldID(cls, name, "D");
        if(field) env->SetDoubleField(snapshotObject, field, value);
    };

    setBoolean("initialized", snapshot.initialized ? JNI_TRUE : JNI_FALSE);
    setBoolean("running", snapshot.running ? JNI_TRUE : JNI_FALSE);
    setBoolean("paused", snapshot.paused ? JNI_TRUE : JNI_FALSE);
    setInt("runnerState", snapshot.runner_state);
    setLong("frameId", static_cast<jlong>(snapshot.frame_id));
    setDouble("timestamp", snapshot.timestamp);
    setInt("trackingState", snapshot.tracking_state);
    setBoolean("hasPose", snapshot.has_pose ? JNI_TRUE : JNI_FALSE);
    setBoolean("hasDynamicMask", snapshot.has_dynamic_mask ? JNI_TRUE : JNI_FALSE);
    setBoolean("hasEstimatedDepth", snapshot.has_estimated_depth ? JNI_TRUE : JNI_FALSE);
    setBoolean("trackingOk", snapshot.tracking_ok ? JNI_TRUE : JNI_FALSE);
    setBoolean("requestedNewKeyframe", snapshot.requested_new_keyframe ? JNI_TRUE : JNI_FALSE);
    setBoolean("isKeyframe", snapshot.is_keyframe ? JNI_TRUE : JNI_FALSE);
    setLong("trackedKeypointCount", static_cast<jlong>(snapshot.tracked_keypoint_count));
    setLong("trackedMapPointCount", static_cast<jlong>(snapshot.tracked_map_point_count));
    setLong("activeMapKeyframeCount", static_cast<jlong>(snapshot.active_map_keyframe_count));
    setLong("activeMapPointCount", static_cast<jlong>(snapshot.active_map_point_count));

    jfieldID poseField = env->GetFieldID(cls, "poseMatrix", "[F");
    if(poseField)
    {
        jobject existing = env->GetObjectField(snapshotObject, poseField);
        jfloatArray poseArray = static_cast<jfloatArray>(existing);
        if(!poseArray || env->GetArrayLength(poseArray) != 16)
        {
            poseArray = env->NewFloatArray(16);
            env->SetObjectField(snapshotObject, poseField, poseArray);
        }
        if(poseArray)
            env->SetFloatArrayRegion(poseArray, 0, 16, snapshot.pose_matrix);
    }

    return true;
}

mdslam_handle* FromHandle(jlong handle)
{
    return reinterpret_cast<mdslam_handle*>(static_cast<intptr_t>(handle));
}

jlong JNICALL NativeCreateSession(JNIEnv*, jobject)
{
    return reinterpret_cast<jlong>(mdslam_create());
}

void JNICALL NativeDestroySession(JNIEnv*, jobject, jlong handle)
{
    mdslam_destroy(FromHandle(handle));
}

jboolean JNICALL NativeInitialize(JNIEnv* env, jobject, jlong handle, jstring vocabularyPath, jstring settingsPath,
                                  jboolean useViewer, jint initFrame, jstring sequenceName, jint sensorMode)
{
    mdslam_session_config config = {};
    std::string vocabulary = JStringToStdString(env, vocabularyPath);
    std::string settings = JStringToStdString(env, settingsPath);
    std::string sequence = JStringToStdString(env, sequenceName);
    config.vocabulary_path = vocabulary.c_str();
    config.settings_path = settings.c_str();
    config.use_viewer = useViewer ? 1 : 0;
    config.init_frame = initFrame;
    config.sequence_name = sequence.empty() ? nullptr : sequence.c_str();
    config.sensor_mode = sensorMode;
    return mdslam_initialize(FromHandle(handle), &config) ? JNI_TRUE : JNI_FALSE;
}

void JNICALL NativeShutdown(JNIEnv*, jobject, jlong handle)
{
    mdslam_shutdown(FromHandle(handle));
}

jboolean JNICALL NativeOpenVideo(JNIEnv* env, jobject, jlong handle, jstring source, jboolean realtimePlayback)
{
    mdslam_video_config config = {};
    std::string src = JStringToStdString(env, source);
    config.source = src.c_str();
    config.realtime_playback = realtimePlayback ? 1 : 0;
    return mdslam_open_video(FromHandle(handle), &config) ? JNI_TRUE : JNI_FALSE;
}

jboolean JNICALL NativeStepVideo(JNIEnv*, jobject, jlong handle)
{
    return mdslam_step_video(FromHandle(handle)) ? JNI_TRUE : JNI_FALSE;
}

void JNICALL NativeFinalizeVideo(JNIEnv*, jobject, jlong handle)
{
    mdslam_finalize_video(FromHandle(handle));
}

jboolean JNICALL NativeProcessFrameBgr(JNIEnv* env, jobject, jlong handle, jobject byteBuffer, jint width, jint height,
                                       jint strideBytes, jdouble timestamp, jstring frameName)
{
    if(!byteBuffer)
        return JNI_FALSE;

    auto* data = static_cast<unsigned char*>(env->GetDirectBufferAddress(byteBuffer));
    if(!data)
        return JNI_FALSE;

    std::string name = JStringToStdString(env, frameName);
    mdslam_frame frame = {};
    frame.bgr_data = data;
    frame.width = width;
    frame.height = height;
    frame.stride_bytes = strideBytes;
    frame.timestamp = timestamp;
    frame.frame_name = name.empty() ? nullptr : name.c_str();
    return mdslam_process_frame_bgr8(FromHandle(handle), &frame) ? JNI_TRUE : JNI_FALSE;
}

jboolean JNICALL NativeProcessFrameRgba(JNIEnv* env, jobject, jlong handle, jobject byteBuffer, jint width, jint height,
                                        jint strideBytes, jdouble timestamp, jstring frameName)
{
    if(!byteBuffer)
        return JNI_FALSE;

    auto* data = static_cast<unsigned char*>(env->GetDirectBufferAddress(byteBuffer));
    if(!data)
        return JNI_FALSE;

    cv::Mat rgba(height, width, CV_8UC4, data, static_cast<size_t>(strideBytes));
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

    std::string name = JStringToStdString(env, frameName);
    mdslam_frame frame = {};
    frame.bgr_data = bgr.data;
    frame.width = bgr.cols;
    frame.height = bgr.rows;
    frame.stride_bytes = static_cast<int>(bgr.step);
    frame.timestamp = timestamp;
    frame.frame_name = name.empty() ? nullptr : name.c_str();
    return mdslam_process_frame_bgr8(FromHandle(handle), &frame) ? JNI_TRUE : JNI_FALSE;
}

jboolean JNICALL NativeGetSnapshot(JNIEnv* env, jobject, jlong handle, jobject snapshotObject)
{
    mdslam_snapshot snapshot = {};
    if(!mdslam_get_snapshot(FromHandle(handle), &snapshot))
        return JNI_FALSE;
    return FillSnapshotObject(env, snapshotObject, snapshot) ? JNI_TRUE : JNI_FALSE;
}

jlong JNICALL NativeCreateAnchorAtCurrentPose(JNIEnv* env, jobject, jlong handle, jstring label)
{
    std::string anchorLabel = JStringToStdString(env, label);
    return static_cast<jlong>(mdslam_create_anchor_at_current_pose(FromHandle(handle), anchorLabel.empty() ? nullptr : anchorLabel.c_str()));
}

void JNICALL NativeClearAnchors(JNIEnv*, jobject, jlong handle)
{
    mdslam_clear_anchors(FromHandle(handle));
}

jstring JNICALL NativeGetLastError(JNIEnv* env, jobject, jlong handle)
{
    return StdStringToJString(env, mdslam_get_last_error(FromHandle(handle)));
}

static const JNINativeMethod kMethods[] = {
    {"nativeCreateSession", "()J", reinterpret_cast<void*>(NativeCreateSession)},
    {"nativeDestroySession", "(J)V", reinterpret_cast<void*>(NativeDestroySession)},
    {"nativeInitialize", "(JLjava/lang/String;Ljava/lang/String;ZILjava/lang/String;I)Z", reinterpret_cast<void*>(NativeInitialize)},
    {"nativeShutdown", "(J)V", reinterpret_cast<void*>(NativeShutdown)},
    {"nativeOpenVideo", "(JLjava/lang/String;Z)Z", reinterpret_cast<void*>(NativeOpenVideo)},
    {"nativeStepVideo", "(J)Z", reinterpret_cast<void*>(NativeStepVideo)},
    {"nativeFinalizeVideo", "(J)V", reinterpret_cast<void*>(NativeFinalizeVideo)},
    {"nativeProcessFrameBgr", "(JLjava/nio/ByteBuffer;IIIDLjava/lang/String;)Z", reinterpret_cast<void*>(NativeProcessFrameBgr)},
    {"nativeProcessFrameRgba", "(JLjava/nio/ByteBuffer;IIIDLjava/lang/String;)Z", reinterpret_cast<void*>(NativeProcessFrameRgba)},
    {"nativeGetSnapshot", "(JLcom/timjaung/monocularslam/MdslamSnapshot;)Z", reinterpret_cast<void*>(NativeGetSnapshot)},
    {"nativeCreateAnchorAtCurrentPose", "(JLjava/lang/String;)J", reinterpret_cast<void*>(NativeCreateAnchorAtCurrentPose)},
    {"nativeClearAnchors", "(J)V", reinterpret_cast<void*>(NativeClearAnchors)},
    {"nativeGetLastError", "(J)Ljava/lang/String;", reinterpret_cast<void*>(NativeGetLastError)},
};

} // namespace

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*)
{
    JNIEnv* env = nullptr;
    if(vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK || !env)
        return JNI_ERR;

    jclass nativeClass = env->FindClass(kNativeClassName);
    if(!nativeClass)
        return JNI_ERR;

    if(env->RegisterNatives(nativeClass, kMethods, sizeof(kMethods) / sizeof(kMethods[0])) != JNI_OK)
        return JNI_ERR;

    jclass snapshotClass = env->FindClass(kSnapshotClassName);
    if(!snapshotClass)
        return JNI_ERR;

    return JNI_VERSION_1_6;
}
