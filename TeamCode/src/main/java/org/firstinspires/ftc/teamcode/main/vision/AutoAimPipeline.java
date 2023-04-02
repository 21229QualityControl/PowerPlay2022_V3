package org.firstinspires.ftc.teamcode.main.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/*
Based off of 13356 RoboForce's implementation
https://www.youtube.com/watch?v=rQjcZt6V9ac&ab_channel=FTCRoboForce
https://github.com/ftc13356/PowerPlay/blob/c5c617fa7f994e29f8d133fb818af8c2496a64ff/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Components/CV/StickObserverPipeline.java#L140
 */
@Config
public class AutoAimPipeline extends Pipeline {

    /*
    Ideas:
    - Program that measures lighting and remaps consistently
    - Remap LUTs stored in memory
    - Compare to localization and angled ray in front of turret
    -

     */

    public static DisplayTypes DISPLAY_MODE = DisplayTypes.RAW_0;

    public enum DisplayTypes {
        RAW_0,
        LENIENT_THRESH_1,
        LENIENT_MASK_2,
        SCALED_MASK_3,
        STRICT_THRESH_4,
        STRICT_MASK_5,
        EDGES_6,

    }

    private final Scalar lenientLowHSV = new Scalar(20, 70, 80);
    private final Scalar lenientHighHSV = new Scalar(32, 255, 255);

    private final Scalar strictLowSat = new Scalar(0, 140, 0);
    private final Scalar strictHighSat = new Scalar(255, 255, 255);

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Lenient threshold to keep all yellow objects
        Mat thresh = new Mat();
        Core.inRange(hsv, lenientLowHSV, lenientHighHSV, thresh);
        Mat masked = new Mat();
        Core.bitwise_and(hsv, hsv, masked, thresh);

        // Scale to average Saturation
        Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150/average.val[1], 0); // scales avg to 150

        // Strict threshold to the brighter yellows
        Mat strictScaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowSat, strictHighSat, strictScaledThresh);
        Mat strictScaledMask = new Mat();
        Core.bitwise_and(hsv, hsv, strictScaledMask, strictScaledThresh);

        // Detect edges (not necessary)
        Mat edges = new Mat();
        Imgproc.Canny(strictScaledMask, edges, 100, 200);

        // Find contours
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(strictScaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat output;
        switch (DISPLAY_MODE) {
            default:
                output = input;
                break;
            case LENIENT_THRESH_1:
                output = thresh;
                break;
            case LENIENT_MASK_2:
                output = new Mat();
                Imgproc.cvtColor(masked, output, Imgproc.COLOR_HSV2RGB);
                break;
            case SCALED_MASK_3:
                output = new Mat();
                Imgproc.cvtColor(scaledMask, output, Imgproc.COLOR_HSV2RGB);
                break;
            case STRICT_THRESH_4:
                output = strictScaledThresh;
                break;
            case STRICT_MASK_5:
                output = new Mat();
                Imgproc.cvtColor(strictScaledMask, output, Imgproc.COLOR_HSV2RGB);
                break;
            case EDGES_6:
                output = edges;
                break;
        }

        if (output != thresh) thresh.release();
        if (output != masked) masked.release();
        if (output != scaledMask) scaledMask.release();
        if (output != strictScaledMask) strictScaledMask.release();
        if (output != strictScaledThresh) strictScaledThresh.release();
        if (output != edges) edges.release();

        return output;
    }
}
