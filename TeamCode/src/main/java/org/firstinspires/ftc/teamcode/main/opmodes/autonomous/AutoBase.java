package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.HEADING_PID;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.main.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.main.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.main.subsystems.Hub;
import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.LED;
import org.firstinspires.ftc.teamcode.main.subsystems.Memory;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.main.subsystems.Vision;
import org.firstinspires.ftc.teamcode.roadrunner.PositionMaintainer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StateCopyLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

public abstract class AutoBase extends LinearOpMode {
    // subsystems
    protected Drivetrain drivetrain;
    protected Roadrunner rr;
    protected Hub hub;
    protected Intake intake;
    protected Outtake outtake;
    protected Vision vision;
    protected LED led;

    // controllers. WARNING: you are not supposed to touch the robot/controllers after init
    private GamePadController g1;
    private GamePadController g2;

    // just for park
    private PositionMaintainer parker;

    // config variables
    public static double START_DELAY = 0; // Seconds to wait before running
    public static int SIGNAL = -1; // Field Setup scenario

    // config switches
    public static boolean SIGNAL_OVERRIDE = false;

    // default values
    private final int defaultSignal = 2; // middle

    final public void runOpMode() throws InterruptedException {
        /* ********** INIT ********** */
        // telemetry init
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        // setup
        Dashboard.setUp();
        Memory.IS_BLUE = isBlue();
        Memory.AUTO_START_POSE = getStartPose();
        START_DELAY = 0;

        // instantiate subsystems
        this.hub = new Hub(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap, hub);
        this.rr = new Roadrunner(hardwareMap,hub, drivetrain);
        this.intake = new Intake(hardwareMap);
        this.outtake = new Outtake(hardwareMap);
        Memory.REMEMBERED_OUTTAKE = this.outtake;
        this.vision = new Vision(hardwareMap);
        this.led = new LED(hardwareMap);

        this.parker = new PositionMaintainer(new PIDCoefficients(3, 0, 0), new PIDCoefficients(3, 0, 0), HEADING_PID, new Pose2d(0.3, 0.3, Math.toRadians(1)));

        // draw intake
        this.rr.getTrajectorySequenceRunner().dashboardConsumers.add((canvas) -> {
            canvas.setStroke("#4CAF50");
            DashboardUtil.drawIntake(canvas, rr.getPoseEstimate(), intake.extenderTicksToInches(intake.getExtenderTarget()));

            canvas.setStroke("#3F51B5");
            DashboardUtil.drawIntake(canvas, rr.getPoseEstimate(), intake.extenderTicksToInches(intake.getExtenderPosition()));
        });
        // draw outtake
        this.rr.getTrajectorySequenceRunner().dashboardConsumers.add((canvas) -> {
            canvas.setStroke("#4CAF50");
            DashboardUtil.drawOuttake(canvas, rr.getPoseEstimate(), Math.toRadians(outtake.getTurretTarget()), outtake.slideTicksToInches(outtake.getSlideTarget()) * Math.cos(Math.toRadians(Outtake.SLIDE_ANGLE)), outtake.isArmOut());

            canvas.setStroke("#3F51B5");
            DashboardUtil.drawOuttake(canvas, rr.getPoseEstimate(), Math.toRadians(outtake.getTurretAngle()), outtake.slideTicksToInches(outtake.getSlidePosition()) * Math.cos(Math.toRadians(Outtake.SLIDE_ANGLE)), outtake.isArmOut());
        });

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
        while (opModeInInit()) {
            // WARNING: you are not supposed to touch the robot/controllers after init
            // Control start delay
            if (g1.leftStickUpOnce() || g2.leftStickUpOnce()) {
                START_DELAY = Math.max(0, START_DELAY + 0.5);
            } else if (g1.leftStickDownOnce() || g2.leftStickDownOnce()) {
                START_DELAY = Math.max(0, START_DELAY - 0.5);
            }

            // use vision for signal
            vision.updatePolling();
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
            outtake.update();
            idle();
        }

        /* ********** INIT END - AUTO START ********** */

        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence

        vision.stopStreaming(); // Takes 0.135 seconds

        if (isStopRequested()) return; // exit if stopped

        // use last detection or default signal if last cycle failed
        if (SIGNAL == -1) {
            SIGNAL = vision.getLastValidReading();
            if (SIGNAL == -1) SIGNAL = defaultSignal;
        }
        Log.d("Autonomous", "Signal used: " + SIGNAL);

        // set pose again just in case
        rr.setPoseEstimate(getStartPose());

        intake.setExtenderMaxPower(1.0);

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

        // prepare park police
        parker.resetController();
        parker.maintainPosition(StateCopyLocalizer.pose);

        // reset slide encoder
        if (outtake.isSlideMagnetPresent()) {
            Log.d("Outtake", "Reset slide encoders from " + outtake.getSlidePosition());
            outtake.getSlide().zeroMotorInternals();
        } else {
            Log.d("Outtake", "Skipped slide encoder reset due to missing magnet");
        }

        // run user end
        onEnd();

        /* ********** END LOOP ********** */
        do {
            // telemetry end time
            telemetry.addData("End Time", endTime);

            // keep robot on park
            rr.setDriveSignal(parker.update(rr.getPoseEstimate(), rr.getPoseVelocity()));

            // record end pose
            rr.updatePoseEstimate();
            Memory.LAST_POSE = rr.getPoseEstimate();

            // call user end loop
            onEndLoop();

            if (intake.getExtender().getMotor().isOverCurrent()) intake.setExtenderMaxPower(0);

            intake.update();
            outtake.update();
            telemetry.update();
        } while (opModeIsActive() && !isStopRequested());

        // Ensure motors are stopped
        rr.setMotorPowers(0, 0, 0, 0);
        intake.getExtender().stopMotor();
        outtake.getTurret().stopMotor();
        outtake.getSlide().stopMotor();
        rr.forceStopTrajectory();
    }

    private void printStartupStatus() {
        String camStatus;
        if (SIGNAL_OVERRIDE) camStatus = "OVERRIDE " + SIGNAL;
        else if (!vision.pipeline.isInitialized()) camStatus = "INITIALIZING";
        else if (vision.getReading() == -1 && vision.getLastValidReading() != -1) camStatus = "LOST [ " + vision.getLastValidReading() + " ] for " + vision.getFramesWithoutDetection() + " frames";
        else if (vision.getLastValidReading() == -1) camStatus = "FAILED TO READ";
        else camStatus = "FOUND [ " + vision.getReading() + " ]";

        String motorStatus;
        if (Math.abs(outtake.getSlidePosition()) > 1) motorStatus = "OUTTAKE SLIDE NOT RESET";
        else if (Math.abs(intake.getExtenderPosition()) > 1) motorStatus = "INTAKE EXTENDER NOT RESET";
        else if (Math.abs(outtake.getTurretAngle()) > 1) motorStatus = "OUTTAKE TURRET NOT RESET";
        else motorStatus = "READY";

//        String sensorStatus;
//        if (surroundSensors.frontDistance.isTimedout() || surroundSensors.leftDistance.isTimedout() || surroundSensors.rightDistance.isTimedout()) {
//            sensorStatus = "DISTANCE TIMEOUT";
//        } else if (freightSystem.getDetectionDistance() > 5.9) {
//            sensorStatus = "OCCUPANCY FAILURE";
//        } else {
//            sensorStatus = "READY";
//        }

        telemetry.addData("Vision", camStatus);
        telemetry.addData("    Debug", vision.getDebugData());
        telemetry.addData("Motor status", motorStatus);
        telemetry.addData("    Turret Angle", outtake.getTurretAngle() + " -> " + outtake.getTurretTarget());
        telemetry.addData("    Outtake Pos", outtake.getSlidePosition() + " -> " + outtake.getSlideTarget());
        telemetry.addData("    Extender Pos", intake.getExtenderPosition() + " -> " + intake.getExtenderTarget());
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

    protected void initSystem() {
        // display init led
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        // init intake first
        intake.initialize();

        // push slide into their limits
        intake.getExtender().getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.getExtender().getMotor().setPower(-0.3);
        outtake.getSlide().getMainMotor().setPower(-0.2);
        outtake.getSlide().getSecondMotor().setPower(-0.2);
        sleep(1000);

        // stop motors
        intake.getExtender().getMotor().setPower(0);
        outtake.getSlide().getMainMotor().setPower(0);
        outtake.getSlide().getSecondMotor().setPower(0);
        sleep(100);

        // reset the encoders
        intake.getExtender().zeroMotorInternals();
        intake.getExtender().getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.getTurret().setInternalAngle(0);
        outtake.getSlide().zeroMotorInternals();

        // init outtake
        outtake.initialize();

        // disable led
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    /**
     * Iterative method for each follower update call (not for updatePoseEstimate())
     */
    protected void update() {
        Dashboard.packet.put("Runtime", getRuntime());
        Dashboard.packet.put("Turret angle", outtake.getTurretAngle());
        Dashboard.packet.put("Turret target", outtake.getTurretTarget());
        Dashboard.packet.put("Turret power", outtake.getTurretPower());
        telemetry.addData("Time left", 30 - getRuntime());
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
        outtake.update();
        intake.update();
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
