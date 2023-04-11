package org.firstinspires.ftc.teamcode.main.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.main.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/**
 * Class for a Barcode facing camera (left or right)
 */
//@Config
public class Vision {
    /*
     * Supported resolutions for the wide angle camera are
     * [640x480=1.33], [1600x896=1.62], [1280x720=1.78],
     * [1024x768=1.33], [1024x576=1.78], [960x544=1.76],
     * [864x480=1.8], [848x480=1.77], [800x448=1.79],
     * [640x360=1.78], [352x288=1.22], [320x240=1.33], [1920x1080=1.78],
     * From testing, bigger resolutions and wider resolutions are terrible at frame rate
     */
    private static int WIDTH = 320; // wide strip: 640x360
    private static int HEIGHT = 240;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    private double tagsize = 0.166;

    private OpenCvWebcam webcam;
    public AprilTagDetectionPipeline pipeline;

    private int lastReading = -1;
    private int lastValidReading = -1;

    private int numFramesWithoutDetection = 0;

    private final float DECIMATION_HIGH = 3;
    private final float DECIMATION_LOW = 2;
    private final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.e("Vision", "openCameraDeviceAsync() failed - Error code " + errorCode);
            }
        });

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(pipeline);

        Dashboard.startCameraStream(webcam, 60);
    }

    /**
     * Must be called to refresh our reading, even though detection pipeline is on a separate thread
     */
    public void updatePolling() {
        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

        // Check if there's a new frame (detectionsUpdate is null after using its frame)
        if (detections != null) {
            if (detections.size() == 0) { // check if there's no tag detected
                numFramesWithoutDetection++;

                lastReading = -1;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    pipeline.setDecimation(DECIMATION_LOW);
                }
            } else { // if we have a tag
                numFramesWithoutDetection = 0;

                lastReading = mapAprilTagToSignal(detections);

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    pipeline.setDecimation(DECIMATION_HIGH);
                }
            }
        }

        if (lastReading != -1) {
            lastValidReading = lastReading;
        }
    }

    private int mapAprilTagToSignal(ArrayList<AprilTagDetection> aprilTags) {
        int val;
        for (AprilTagDetection tag : aprilTags) {
            val = mapAprilTagToSignal(tag.id);
            if (val != -1) return val;
        }
        return -1;
    }

    private int mapAprilTagToSignal(int aprilTagID) {
        switch (aprilTagID) {
            case 8:
                return 1;
            case 18:
                return 2;
            case 28:
                return 3;
            default:
                return -1;
        }
    }

    public String getDebugData() {
        return String.format("FPS:%.0f/%d | ms:%d = %d + %d", webcam.getFps(), webcam.getCurrentPipelineMaxFps(), webcam.getTotalFrameTimeMs(), webcam.getPipelineTimeMs(), webcam.getOverheadTimeMs());
    }

    public int getReading() {
        return lastReading;
    }

    public int getLastValidReading() {
        return lastValidReading;
    }

    public int getFramesWithoutDetection() {
        return numFramesWithoutDetection;
    }

    public void close() {
        try {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        } catch (OpenCvCameraException e) {
            // ignore, don't crash the app
        }
    }
    public void stopStreaming() {
        try {
            webcam.stopStreaming();
            Dashboard.stopCameraStream();
        } catch (OpenCvCameraException e) {
            // ignore, don't crash the app
        }
    }
}
