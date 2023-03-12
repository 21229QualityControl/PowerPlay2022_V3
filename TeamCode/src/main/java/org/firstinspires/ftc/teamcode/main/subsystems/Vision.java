package org.firstinspires.ftc.teamcode.main.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.main.vision.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Class for a Barcode facing camera (left or right)
 */
@Config
public class Vision {
    /*
     * Supported resolutions for the wide angle camera are
     * [640x480=1.33], [1600x896=1.62], [1280x720=1.78],
     * [1024x768=1.33], [1024x576=1.78], [960x544=1.76],
     * [864x480=1.8], [848x480=1.77], [800x448=1.79],
     * [640x360=1.78], [352x288=1.22], [320x240=1.33], [1920x1080=1.78],
     * From testing, bigger resolutions and wider resolutions are terrible at frame rate
     */
    public static int WIDTH = 320; // wide strip: 640x360
    public static int HEIGHT = 240;

    private OpenCvWebcam webcam;
    public Pipeline activePipeline;

    private int lastValidReading = -1;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.e("Vision", "openCameraDeviceAsync() failed " + errorCode);
            }
        });

        Dashboard.getInstance().startCameraStream(webcam, 60);
    }

    // TODO: Your pipeline should extend Pipeline
    public void setPipeline(Pipeline pipeline) {
        activePipeline = pipeline;
        webcam.setPipeline(activePipeline);
    }

    public String getDebugData() {
        return String.format("FPS:%.0f/%d | ms:%d = %d + %d", webcam.getFps(), webcam.getCurrentPipelineMaxFps(), webcam.getTotalFrameTimeMs(), webcam.getPipelineTimeMs(), webcam.getOverheadTimeMs());
    }

    public int getReading() {
        if (activePipeline == null || !activePipeline.isInitialized()) return -1;
        int reading = activePipeline.getReading();
        if (reading != -1) this.lastValidReading = reading;
        return activePipeline.getReading();
    }

    public int getLastValidReadingOrDefault(int defaultReading) {
        if (this.lastValidReading == -1) return defaultReading;
        return this.lastValidReading;
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
        } catch (OpenCvCameraException e) {
            // ignore, don't crash the app
        }
    }
}
