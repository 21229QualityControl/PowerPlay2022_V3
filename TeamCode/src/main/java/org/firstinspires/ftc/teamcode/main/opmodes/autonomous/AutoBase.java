package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.main.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.main.subsystems.Hub;
import org.firstinspires.ftc.teamcode.main.subsystems.Memory;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.main.subsystems.Vision;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public abstract class AutoBase extends LinearOpMode {
    // subsystems
    protected Drivetrain drivetrain;
    protected Roadrunner rr;
    protected Hub hub;
    protected Vision vision;

    // controllers. WARNING: you are not supposed to touch the robot/controllers after init
    private GamePadController g1;
    private GamePadController g2;

    // config variables
    public static double START_DELAY = 0; // Seconds to wait before running
    public static int SIGNAL = -1; // Field Setup scenario

    // config switches
    public static boolean SIGNAL_OVERRIDE = false;

    // default values
    private final int defaultSignal = 18; // middle

    final public void runOpMode() throws InterruptedException {
        /* ********** INIT ********** */
        // telemetry init
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        // setup
        Dashboard.setUp();
        Memory.IS_BLUE = isBlue();
        START_DELAY = 0;

        // instantiate subsystems
        this.hub = new Hub(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap, hub);
        this.rr = new Roadrunner(hardwareMap,hub, drivetrain);
        this.vision = new Vision(hardwareMap);

        // setup gamepad controls
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);

        // set auto pose
        rr.setPoseEstimate(getStartPose());
        rr.update(); // ensure one update has gone through

        // init intake and outtake
        initSystem();

        // run user init
        onInit();

        /* ********** INIT LOOP ********** */
        while (!isStarted() && !isStopRequested()) {
            // WARNING: you are not supposed to touch the robot/controllers after init
            // Control start delay
            if (g1.leftStickUpOnce() || g2.leftStickUpOnce()) {
                START_DELAY = Math.max(0, START_DELAY + 0.5);
            } else if (g1.leftStickDownOnce() || g2.leftStickDownOnce()) {
                START_DELAY = Math.max(0, START_DELAY - 0.5);
            }

            // use vision for signal
            if (!SIGNAL_OVERRIDE) SIGNAL = vision.getReading();

            // add telemetry info sheet
            printDescription();
            printStartupStatus();
            printInstructions();
            displayLED();

            // run user init loop
            onInitLoop();

            // update loop
            telemetry.update();
            Dashboard.sendPacket();
            g1.update();
            g2.update();
            idle();
        }

        /* ********** INIT END - AUTO START ********** */

//        vision.stopStreaming(); // TODO: Disable cam stream

        if (isStopRequested()) return; // exit if stopped

        resetRuntime(); // reset runtime timer

        // use last detection or default signal if last cycle failed
        if (SIGNAL == -1) {
            SIGNAL = vision.getLastValidReadingOrDefault(defaultSignal);
        }

        // set pose again just in case
        rr.setPoseEstimate(getStartPose());

        // run user start
        onStart();

        /* ********** START DELAY ********** */
        if (START_DELAY > 0) {
            follow(builder()
                    .waitSeconds(START_DELAY)
                    .build());
        }

        /* ********** START RUN ********** */
        // run user's main auto code
        onRun();

        /* ********** RUN END ********** */
        // log end time
        double endTime = getRuntime();
        Log.d("Auto", "Auto ended at " + endTime);

        // run user end
        onEnd();

        /* ********** END LOOP ********** */
        while (opModeIsActive() && !isStopRequested()) {
            // telemetry end time
            telemetry.addData("End Time", endTime);

            // keep robot still
            rr.setMotorPowers(0, 0, 0, 0);

            // record end pose
            rr.updatePoseEstimate();
            Memory.LAST_POSE = rr.getPoseEstimate();

            // call user end loop
            onEndLoop();

            telemetry.update();
        }
    }

    private void printStartupStatus() {
//        String camStatus;
//        if (!barcodeCam.activePipeline.isInitialized()) camStatus = "INITIALIZING";
//        else if (BARCODE == -1 && lastValidBarcode != -1) camStatus = "LOST [ " + lastValidBarcode + "]";
//        else if (BARCODE == -1) camStatus = "FAILED TO READ";
//        else if (BARCODE < 1 || 3 < BARCODE) camStatus = "UNKNOWN ERROR ( " + BARCODE + " )"; // this shouldn't happen
//        else {
//            camStatus = "FOUND [ " + BARCODE + " ]"; // 1=Left, 2=Mid, 3=Right
//        }
//
//        String sensorStatus;
//        if (surroundSensors.frontDistance.isTimedout() || surroundSensors.leftDistance.isTimedout() || surroundSensors.rightDistance.isTimedout()) {
//            sensorStatus = "DISTANCE TIMEOUT";
//        } else if (freightSystem.getDetectionDistance() > 5.9) {
//            sensorStatus = "OCCUPANCY FAILURE";
//        } else {
//            sensorStatus = "READY";
//        }
//
//        telemetry.addData("Camera", "camStatus not implemented");
//        telemetry.addData("    Gamma Value", CAMERA_GAMMA_VALUE);
//        telemetry.addData("Sensors", sensorStatus);
//        telemetry.addData("    Occupancy", freightSystem.getDetectionDistance());
//        telemetry.addData("    FrontDist", (surroundSensors.frontDistance.isTimedout() ? "TIMEOUT - " : "") + String.format("%.2f", surroundSensors.getFrontDist()));
//        telemetry.addData("    LeftDist", (surroundSensors.leftDistance.isTimedout() ? "TIMEOUT - " : "") + String.format("%.2f", surroundSensors.getLeftDist()));
//        telemetry.addData("    RightDist", (surroundSensors.rightDistance.isTimedout() ? "TIMEOUT - " : "") + String.format("%.2f", surroundSensors.getRightDist()));

        telemetry.addData("Delay", START_DELAY == 0 ? "NONE" : START_DELAY);
    }

    private void printInstructions() {
        telemetry.addLine();
        telemetry.addLine("Left Stick Up/Down: Change start delay");
    }

    private void displayLED() {
//        switch (SIGNAL) {
//            case 1:
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
//                break;
//            case 2:
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
//                break;
//            case 3:
//                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
//                break;
//            default:
//                switch (vision.getLastDetectionId()) {
//                    case 1:
//                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
//                        break;
//                    case 2:
//                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
//                        break;
//                    case 3:
//                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
//                        break;
//                    default:
//                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
//                        break;
//                }
//        }
    }

    private void handleVisionSettings() {
//        // vision settings
//        if (g1.aOnce()) AprilTagDetectionPipeline.WHITE_BALANCE_IMAGE = !AprilTagDetectionPipeline.WHITE_BALANCE_IMAGE;
//        if (g1.rightStickUpOnce()) CAMERA_GAMMA_VALUE /= 1.1; // brighten
//        if (g1.rightStickDownOnce()) CAMERA_GAMMA_VALUE *= 1.1; // darken
//        vision.aprilTagDetectionPipeline.setGammaValue(CAMERA_GAMMA_VALUE);
    }

    protected void initSystem() {
        // TODO: implement auto init system, which runs before every autonomous
    }

    /**
     * Iterative method for each follower update call (not for updatePoseEstimate())
     */
    protected void update() {
        Dashboard.packet.put("Runtime", getRuntime());
    }

    // Settings that initialization needs
    protected abstract boolean isBlue();
    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();

    // Run methods to override
    protected void onInit() {}
    protected void onInitLoop() {}
    protected void onStart() {}
    protected abstract void onRun();
    protected void onEnd() {}
    protected void onEndLoop() {}

    // Updating roadrunner functions
    final protected void followAsync(TrajectorySequence trajSeq) {
        try {
            Log.d("Roadrunner", "Follow Trajectory Sequence async from " + trajSeq.start() + " to " + trajSeq.end() + ", for " + trajSeq.duration() + " seconds");
            rr.followTrajectorySequenceAsync(trajSeq);
        } catch (EmptyPathSegmentException | NullPointerException e) {
            Log.d("Roadrunner", "Follow Trajectory Sequence async FAILED");
            e.printStackTrace();
        }
    }
    final protected void follow(TrajectorySequence trajSeq) {
        try {
            Log.d("Roadrunner", "Follow Trajectory Sequence from " + trajSeq.start() + " to " + trajSeq.end() + ", for " + trajSeq.duration() + " seconds");
            rr.followTrajectorySequence(trajSeq);
        } catch (EmptyPathSegmentException | NullPointerException e) {
            Log.d("Roadrunner", "Follow Trajectory Sequence FAILED");
            e.printStackTrace();
        }
    }
    final protected TrajectorySequenceBuilder builder(Pose2d startPose) {
        return rr.trajectorySequenceBuilder(startPose).addIterative("update", this::update);
    }
    final protected TrajectorySequenceBuilder builder() {
        return rr.trajectorySequenceBuilder(rr.getPoseEstimate()).addIterative("update", this::update);
    }
    final protected void waitSeconds(double seconds) {
        Log.d("Roadrunner", "Wait " + seconds + " seconds");
        follow(builder().waitSeconds(seconds).build());
    }
}
