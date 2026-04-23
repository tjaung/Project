package com.timjaung.monocularslam

import java.io.Closeable
import java.nio.ByteBuffer

class MdslamSnapshot {
    @JvmField var initialized: Boolean = false
    @JvmField var running: Boolean = false
    @JvmField var paused: Boolean = false
    @JvmField var runnerState: Int = 0
    @JvmField var frameId: Long = 0L
    @JvmField var timestamp: Double = 0.0
    @JvmField var trackingState: Int = -1
    @JvmField var hasPose: Boolean = false
    @JvmField var hasDynamicMask: Boolean = false
    @JvmField var hasEstimatedDepth: Boolean = false
    @JvmField var trackingOk: Boolean = false
    @JvmField var requestedNewKeyframe: Boolean = false
    @JvmField var isKeyframe: Boolean = false
    @JvmField var trackedKeypointCount: Long = 0L
    @JvmField var trackedMapPointCount: Long = 0L
    @JvmField var activeMapKeyframeCount: Long = 0L
    @JvmField var activeMapPointCount: Long = 0L
    @JvmField var poseMatrix: FloatArray = FloatArray(16)
}

class MonocularSlamNative {
    external fun nativeCreateSession(): Long
    external fun nativeDestroySession(handle: Long)
    external fun nativeInitialize(
        handle: Long,
        vocabularyPath: String,
        settingsPath: String,
        useViewer: Boolean,
        initFrame: Int,
        sequenceName: String?,
        sensorMode: Int
    ): Boolean
    external fun nativeShutdown(handle: Long)
    external fun nativeOpenVideo(handle: Long, source: String, realtimePlayback: Boolean): Boolean
    external fun nativeStepVideo(handle: Long): Boolean
    external fun nativeFinalizeVideo(handle: Long)
    external fun nativeProcessFrameBgr(
        handle: Long,
        buffer: ByteBuffer,
        width: Int,
        height: Int,
        strideBytes: Int,
        timestamp: Double,
        frameName: String?
    ): Boolean
    external fun nativeProcessFrameRgba(
        handle: Long,
        buffer: ByteBuffer,
        width: Int,
        height: Int,
        strideBytes: Int,
        timestamp: Double,
        frameName: String?
    ): Boolean
    external fun nativeGetSnapshot(handle: Long, snapshot: MdslamSnapshot): Boolean
    external fun nativeCreateAnchorAtCurrentPose(handle: Long, label: String?): Long
    external fun nativeClearAnchors(handle: Long)
    external fun nativeGetLastError(handle: Long): String

    companion object {
        init {
            System.loadLibrary("monocular_slam_jni")
        }

        const val SENSOR_MONOCULAR: Int = 0
        const val SENSOR_IMU_MONOCULAR: Int = 3

        const val RUNNER_IDLE: Int = 0
        const val RUNNER_RUNNING: Int = 1
        const val RUNNER_PAUSED: Int = 2
        const val RUNNER_FINISHED: Int = 3
        const val RUNNER_FAILED: Int = 4
    }
}

class MonocularSlamSession(
    private val native: MonocularSlamNative = MonocularSlamNative()
) : Closeable {
    private var handle: Long = native.nativeCreateSession()

    fun initialize(
        vocabularyPath: String,
        settingsPath: String,
        useViewer: Boolean = false,
        initFrame: Int = 0,
        sequenceName: String? = null,
        sensorMode: Int = MonocularSlamNative.SENSOR_MONOCULAR
    ): Boolean {
        ensureOpen()
        return native.nativeInitialize(
            handle,
            vocabularyPath,
            settingsPath,
            useViewer,
            initFrame,
            sequenceName,
            sensorMode
        )
    }

    fun shutdown() {
        if (handle != 0L) {
            native.nativeShutdown(handle)
        }
    }

    fun openVideo(source: String, realtimePlayback: Boolean = true): Boolean {
        ensureOpen()
        return native.nativeOpenVideo(handle, source, realtimePlayback)
    }

    fun stepVideo(): Boolean {
        ensureOpen()
        return native.nativeStepVideo(handle)
    }

    fun finalizeVideo() {
        if (handle != 0L) {
            native.nativeFinalizeVideo(handle)
        }
    }

    fun processFrameRgba(
        rgbaBuffer: ByteBuffer,
        width: Int,
        height: Int,
        strideBytes: Int,
        timestamp: Double,
        frameName: String? = null
    ): Boolean {
        ensureOpen()
        return native.nativeProcessFrameRgba(handle, rgbaBuffer, width, height, strideBytes, timestamp, frameName)
    }

    fun processFrameBgr(
        bgrBuffer: ByteBuffer,
        width: Int,
        height: Int,
        strideBytes: Int,
        timestamp: Double,
        frameName: String? = null
    ): Boolean {
        ensureOpen()
        return native.nativeProcessFrameBgr(handle, bgrBuffer, width, height, strideBytes, timestamp, frameName)
    }

    fun getSnapshot(snapshot: MdslamSnapshot = MdslamSnapshot()): MdslamSnapshot? {
        ensureOpen()
        return if (native.nativeGetSnapshot(handle, snapshot)) snapshot else null
    }

    fun createAnchorAtCurrentPose(label: String? = null): Long {
        ensureOpen()
        return native.nativeCreateAnchorAtCurrentPose(handle, label)
    }

    fun clearAnchors() {
        if (handle != 0L) {
            native.nativeClearAnchors(handle)
        }
    }

    fun getLastError(): String {
        return if (handle != 0L) native.nativeGetLastError(handle) else "Session is closed"
    }

    override fun close() {
        if (handle != 0L) {
            try {
                native.nativeShutdown(handle)
            } finally {
                native.nativeDestroySession(handle)
                handle = 0L
            }
        }
    }

    private fun ensureOpen() {
        check(handle != 0L) { "MonocularSlamSession is already closed." }
    }
}
