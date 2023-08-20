package org.firstinspires.ftc.teamcode.main.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_HEADING_PID;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TRANSLATIONAL_PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.Cycler_1_Plus_5;
import org.firstinspires.ftc.teamcode.main.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.main.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.main.subsystems.Hub;
import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.LED;
import org.firstinspires.ftc.teamcode.main.subsystems.Memory;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.main.subsystems.SmartGameTimer;
import org.firstinspires.ftc.teamcode.roadrunner.PositionMaintainer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.List;

/**
 * Main Driver program for teleop
 *
 * The controls are below
 *
 * Player 1:
 * - Strafe (Left Stick)
 * - Slow mode Toggle (Left Stick Button, when not keeping position)
 * - Keep Position mode Toggle (Back) (Left Stick Button to disable)
 * - Turns (Bumpers and Triggers)
 * - Grab (A)
 * - Release (B)
 * - Transfer (Y)
 * - Storage (X, only when arm is flat)
 * - Go to intake (X, when arm is angled)
 * - Low junction (Right stick button)
 * - Adjust arm (Right Stick Y)
 * - Adjust wrist (Right Stick X)
 * - Stack adjust (dpad up/down)
 * - Stack auto cycle (A hold)
 * - Extender toggle (Start)
 * - Keep position toggle (Back)
 * - Auto grab/transfer toggle (Guide)
 *
 * Player 2:
 * - Outtake mid (X)
 * - Outtake high (Y)
 * - Retract down (A)
 * - Turret left/mid/right (Left/Right Bumper)
 * - Turret adjustment (Triggers or Right Stick X)
 * - Outtake adjustment (Left Stick Y)
 * -
 */
@Config
@TeleOp(group = "Drive", name = "Manual Drive")
public class ManualDrive extends LinearOpMode {
    public static double SLOW_SPEED = 0.4;
    public static double SLOW_TURN = 0.3;
    public static double SPEED_CONSTANT = 0.9;
    public static double TURN_CONSTANT = 0.75;
    public static double SLEW_RATE = 0.15;
    public static double AUTO_TRANSFER_COOLDOWN = 200; // ms

    public static double BEACON_VSLIDE_POS = 0.38;
    public static double BEACON_ARM_POS = 0.22;
    public static double BEACON_ARM_TIMING = 450;
    public static double BEACON_RELEASE_TIMING = 300;

    private Drivetrain drivetrain;
    private Roadrunner roadrunner;
    private Intake intake;
    private Outtake outtake;
    private Hub hub;
    private Cycler_1_Plus_5 auto;
    private LED led;

    private PositionMaintainer positionMaintainer;
    private ElapsedTime autoTransferTimer;
    private SmartGameTimer smartGameTimer;

    private GamePadController g1, g2;
    private double prevInputx, prevInputy, prevTurn, targetHeading = 0;
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0.25, 0.2);
    private PIDFController headingController = new PIDFController(HEADING_PID, 0, 0, 0);

    private boolean isSlow = false;
    private boolean keepPosition = false;
    private boolean extendIntake = false;
    private boolean autoStack = false;
    private boolean autoTransfer = false;
    private boolean scheduleSlideEncoderReset = false;
    private int stacknum = 1; // 1-5
    private double raisedTurretAngle = 0;

    private boolean warning1 = false;
    private boolean warning2 = false;
    private boolean warning3 = false;
    private boolean warning4 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // On Init
        g1 = new GamePadController(gamepad1);
        g2 = new GamePadController(gamepad2);
        g1.update();
        g2.update();

        Dashboard.setUp();
        hub = new Hub(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, hub);
        roadrunner = new Roadrunner(hardwareMap, hub, drivetrain);
        roadrunner.setPoseEstimate(Memory.LAST_POSE);
        targetHeading = roadrunner.getPoseEstimate().getHeading();
        intake = new Intake(hardwareMap);
        if (Memory.REMEMBERED_OUTTAKE != null) {
            outtake = new Outtake(hardwareMap, Memory.REMEMBERED_OUTTAKE);
            smartGameTimer = new SmartGameTimer(true);
        } else { // WE DON'T HAVE MEMORY OF AUTO!!! we need to align the outtake motors
            outtake = new Outtake(hardwareMap);
            outtake.getSlide().getMainMotor().setPower(-0.3);
            outtake.getSlide().getSecondMotor().setPower(-0.3);
            intake.getExtender().getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.getExtender().getMotor().setPower(-0.3);

            smartGameTimer = new SmartGameTimer(false);
        }

        positionMaintainer = new PositionMaintainer(KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_HEADING_PID, KEEP_POSITION_TOLERANCE.asPose2d());
        led = new LED(hardwareMap);
        auto = new Cycler_1_Plus_5(roadrunner, intake, outtake);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (outtake.isSlideMagnetPresent()) outtake.getSlide().zeroMotorInternals();

        autoTransferTimer = new ElapsedTime();

        // draw intake
        this.roadrunner.getTrajectorySequenceRunner().dashboardConsumers.add((canvas) -> {
            canvas.setStroke("#4CAF50");
            DashboardUtil.drawIntake(canvas, roadrunner.getPoseEstimate(), intake.extenderTicksToInches(intake.getExtenderTarget()));

            canvas.setStroke("#3F51B5");
            DashboardUtil.drawIntake(canvas, roadrunner.getPoseEstimate(), intake.extenderTicksToInches(intake.getExtenderPosition()));
        });
        // draw outtake
        this.roadrunner.getTrajectorySequenceRunner().dashboardConsumers.add((canvas) -> {
            canvas.setStroke("#4CAF50");
            DashboardUtil.drawOuttake(canvas, roadrunner.getPoseEstimate(), Math.toRadians(outtake.getTurretTarget()), outtake.slideTicksToInches(outtake.getSlideTarget()) * Math.cos(Math.toRadians(Outtake.SLIDE_ANGLE)), outtake.isArmOut());

            canvas.setStroke("#3F51B5");
            DashboardUtil.drawOuttake(canvas, roadrunner.getPoseEstimate(), Math.toRadians(outtake.getTurretAngle()), outtake.slideTicksToInches(outtake.getSlidePosition()) * Math.cos(Math.toRadians(Outtake.SLIDE_ANGLE)), outtake.isArmOut());
        });

        // Wait for start
        while (opModeInInit()) {
            g1.update();
            g2.update();
            if (g1.x() || g2.x()) Memory.IS_BLUE = true;
            if (g1.b() || g2.b()) Memory.IS_BLUE = false;

            showGameLights();

            telemetry.addData("ALL Ready !!!", "Press START...");
            telemetry.addData("Color", Memory.IS_BLUE ? "🔵 BLUE" : "🔴 RED");
            telemetry.update();
        }

        // exit immediately if stopped
        if (isStopRequested()) return;

        // On start
        if (opModeIsActive()) {
            resetRuntime();
            g1.reset();
            g2.reset();

            // finish zeroing outtake motors if transition failed
            if (!smartGameTimer.isNominal()) {
                outtake.getSlide().getMainMotor().setPower(0);
                outtake.getSlide().getSecondMotor().setPower(0);
                intake.getExtender().getMotor().setPower(0);
                sleep(20);
                outtake.getSlide().zeroMotorInternals();
                intake.getExtender().zeroMotorInternals();
                intake.getExtender().getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            intake.initialize();
            outtake.initialize();

            smartGameTimer.resetIfStandard();
        }

        // While running
        while (opModeIsActive()) {
            g1.update();
            g2.update();

            move();

            intakeControls();

            outtakeControls();

            warnings();

            sendTelemetry();

            roadrunner.update();
            outtake.update();
            intake.update();
        }

        // On termination
        roadrunner.setMotorPowers(0, 0, 0, 0);
        roadrunner.forceStopTrajectory();
        Memory.LAST_POSE = roadrunner.getPoseEstimate();
    }

    private void sendTelemetry() {
        List<Double> vels = drivetrain.getMotorPowers();
        Dashboard.packet.put("Battery Voltage", hub.getVoltage());
        Dashboard.packet.put("Runtime", smartGameTimer.seconds());
        Dashboard.packet.put("Turret angle", outtake.getTurretAngle());
        telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
        telemetry.addData("Turret Angle", outtake.getTurretAngle() + " -> " + outtake.getTurretTarget());
        telemetry.addData("Outtake Pos", outtake.getSlidePosition() + " -> " + outtake.getSlideTarget());
        telemetry.addData("Extender Pos", intake.getExtenderPosition() + " -> " + intake.getExtenderTarget());
        telemetry.addData("Power", "\n   %4.0f%% | %4.0f%% \n   %4.0f%% | %4.0f%%", vels.get(0)*100, vels.get(3)*100, vels.get(1)*100, vels.get(2)*100);
        telemetry.addData("Position (deg)", Math.toDegrees(roadrunner.getPoseEstimate().getHeading()));
        telemetry.update();
    }

    private double slew(double input, double prev) {
        if (SLEW_RATE < Math.abs(input - prev)) { // Can slew
            if (input < prev) {
                return prev - SLEW_RATE;
            } else if (input > prev) {
                return prev + SLEW_RATE;
            }
        }
        return input; // Close enough that you can just use input
    }

    private void move() {
        // Toggle slow mode, or turn off keepPosition if it is on
//        if (g1.leftStickButtonOnce()) {
//            if (keepPosition) keepPosition = false;
//            else isSlow = !isSlow;
//        }

        // Toggle keep position mode
        if (g1.backLongOnce()) {
            keepPosition = !keepPosition;
            positionMaintainer.resetController();
            positionMaintainer.maintainPosition(roadrunner.getPoseEstimate());
        }

        // Reset angle
        if (g1.backOnce()) {
            roadrunner.setPoseEstimate(new Pose2d(0, 0, 0));
            targetHeading = 0;
        }

        // Movement inputs
        double input_x = Math.pow(-g1.left_stick_y, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_y = Math.pow(-g1.left_stick_x, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_CONSTANT;

        if (g1.leftBumper()) input_turn += SLOW_TURN;
        if (g1.rightBumper()) input_turn -= SLOW_TURN;

        // Set motor powers
        if (keepPosition) {
            // Update maintained position
            Pose2d maintainedPose = positionMaintainer.getMaintainedPosition();
            Pose2d offset = new Pose2d(new Vector2d(input_x * 0.2, input_y * 0.2).rotated(maintainedPose.getHeading()), input_turn * 0.05);
            Pose2d newMaintainedPose = maintainedPose.plus(offset);
            positionMaintainer.maintainPosition(newMaintainedPose);

            roadrunner.setDriveSignal(positionMaintainer.update(roadrunner.getPoseEstimate(), roadrunner.getPoseVelocity()));
        } else {
            // Slew rate limiting
            input_x = slew(input_x, prevInputx);
            input_y = slew(input_y, prevInputy);
            input_turn = slew(input_turn, prevTurn);

            if (input_turn == 0 && prevTurn != 0) { // Need to start keeping position
                targetHeading = roadrunner.getPoseEstimate().getHeading();
            }

            prevInputx = input_x;
            prevInputy = input_y;
            prevTurn = input_turn;

            // Heading PID
            if (input_turn == 0) {
                headingController.setTargetPosition(targetHeading);
                double pidHeading = roadrunner.getPoseEstimate().getHeading();
                if (Math.abs(targetHeading - pidHeading) > Math.toRadians(180)) { // Clip to -180, 180 instead of 0, 360
                    if ((targetHeading - pidHeading) > Math.toRadians(180)) {
                        pidHeading += Math.toRadians(360);
                    } else if ((targetHeading - pidHeading) < Math.toRadians(180)) {
                        pidHeading -= Math.toRadians(360);
                    }
                }
                telemetry.addData("TARGET HEADING", Math.toDegrees(targetHeading));
                telemetry.addData("PID HEADING", Math.toDegrees(pidHeading));
                input_turn = Range.clip(headingController.update(pidHeading), -1, 1);
            }

            // Rotate movement vector
            Vector2d move = new Vector2d(input_x, input_y);
            move = move.rotated(-roadrunner.getPoseEstimate().getHeading());
            roadrunner.setDrivePower(new Pose2d(move.getX(), move.getY(), input_turn));
        }
    }

    private void intakeControls() {
        // Open/Close Claw (A/B)
        if (g1.aOnce()) { // grab
            intake.clawGrab();
        }
        if (g1.bOnce()) { // release
            if (intake.isArmOut()) {
                if (intake.isClawOpen()) intake.clawRelease();
                else intake.clawWide();
            } else {
                intake.clawRelease();
            }
        }

//        // Auto transfer
//        if (g1.guideOnce()) autoTransfer = !autoTransfer;
//        if (autoTransfer && intake.isArmFlat() && intake.isClawOpen() && autoTransferTimer.milliseconds() > AUTO_TRANSFER_COOLDOWN) {
//            if (intake.doesClawSeeCone()) {
//                autoTransferTimer.reset();
//                new Thread(() -> {
//                    intake.clawGrab(); // grab
//                    sleep(100);
//                    intake.liftUp(); // transfer
//                    intake.armIn();
//                    intake.wristInside();
//                    intake.extendTransfer();
//                }).start();
//            }
//        }

        // Go Transfer (Y)
        if (g1.yOnce()) {
            intake.setExtenderMaxPower(0.9);
            intake.extendStore();
            intake.clawGrab();
            outtake.turretCenter();
            outtake.armTransfer();
            intake.vslideTransfer();

            new Thread(() -> { // TODO: Better wait solution
                if (!intake.isExtenderIn()) sleep(250);
                intake.armTransfer();
                sleep(450);
                intake.clawRelease();
                sleep(300);
                intake.vslideDown();
                if (!intake.isArmOut()) intake.armStore();
            }).start();
        }

        // Go Storage or Intake (X)
        if (g1.xOnce()) {
            if (extendIntake) {
                if (intake.getExtenderTarget() > Intake.EXTENDER_BEFORE_CYCLE_TELEOP_POS + 50 && intake.isArmOut()) { // Go Storage
                    intake.setExtenderMaxPower(0.9);
                    intake.extendStore();
                    intake.armStore();
                    intake.vslideDown();
                    if (intake.isClawOpen()) intake.clawRelease();
                } else if (intake.getExtenderTarget() > Intake.EXTENDER_BEFORE_CYCLE_TELEOP_POS - 50) { // Finish extending Intake
                    intake.setExtenderMaxPower(0.6);
                    intake.extendCycleTeleop();
                    intake.armIntake();
                    if (!intake.isClawOpen()) {
                        intake.clawRelease();
                        delayClawWide();
                    }
                } else { // Pre-extend if somehow didn't
                    intake.armIntake();
                    intake.setExtenderMaxPower(0.6);
                    if (intake.getExtenderTarget() < 50) intake.extenderTo(Intake.EXTENDER_BEFORE_CYCLE_TELEOP_POS);
                    if (!intake.isClawOpen()) {
                        intake.clawRelease();
                        delayClawWide();
                    }
                }
            } else {
                if (intake.isArmOut()) { // Go Storage
                    intake.setExtenderMaxPower(0.9);
                    intake.extendStore();
                    intake.armStore();
                    intake.vslideDown();
                    if (intake.isClawOpen()) intake.clawRelease();
                } else { // Go Intake
                    intake.setExtenderMaxPower(0.9);
                    intake.extendStore();
                    intake.armIntake();
                    intake.vslideLevel(1);
                    intake.clawRelease();
                    delayClawWide();
//                autoTransferTimer.reset();
                }
            }
        }

        // Go Low Junction (RightStickButton)
        if (g1.rightStickButtonOnce()) {
            intake.vslideTop();
            intake.armAngledDeposit();
        }

        // Toggle extension
        if (g1.startOnce()) {
            extendIntake = !extendIntake;
            if (extendIntake) {
                intake.setExtenderMaxPower(0.6);
                intake.extendCycleTeleop();
            } else {
                intake.setExtenderMaxPower(0.9);
                intake.extendStore();
            }
        }

        // Adjust arm (RightStickY)
        double armStick = -g1.right_stick_y;
        if (Math.abs(armStick) > 0.01 && Math.abs(armStick) > Math.abs(g1.right_stick_x)) {
            intake.setArmPosition(intake.getArmPosition() + armStick * -0.015);
        }

        // Adjust stack level
        if (g1.dpadUpOnce() || g1.dpadDownOnce()) {
            if (g1.dpadUpOnce()) stacknum++;
            if (g1.dpadDownOnce()) stacknum--;
            stacknum = Range.clip(stacknum, 1, 5);

            if (extendIntake) { // will prepare for auto-cycle only when extending out
                if (!intake.isClawOpen()) {
                    intake.clawRelease();
                    delayClawWide();
                }
                intake.setExtenderMaxPower(0.6);
                intake.extendCycleTeleop();
                intake.armIntake();
            }
            intake.vslideLevel(stacknum);
            autoTransferTimer.reset();
        } else if (g1.dpadRightOnce()) {
            if (intake.isArmOut() && intake.isVSlideUp()) {
                intake.vslideLiftLevel(stacknum);
            }
        } else if (g1.dpadLeftOnce()) {
            intake.armTwoCone();
        }

//        // Auto stack cycle (Has been disabled)
//        if (g1.aLongOnce()) {
//            autoStackCycle();
//        }
    }

    private void outtakeControls() {
        // Outtake high (dpad up)
        if (g2.dpadUpOnce()) {
            if (extendIntake) {
                intake.armIntake();
                intake.setExtenderMaxPower(0.6);
                if (intake.getExtenderTarget() < 50) intake.extenderTo(Intake.EXTENDER_BEFORE_CYCLE_TELEOP_POS);
                if (!intake.isClawOpen()) {
                    intake.clawRelease();
                    delayClawWide();
                }
            } else {
                if (intake.isArmBlockingOuttake()) intake.armStore();
            }

            if (outtake.getSlidePosition() < 50 && !outtake.isArmOut()) outtake.slideHigh();
            else outtake.setSlidePosition(Outtake.SLIDE_HIGH + 50);

            outtake.setTurretAngle(raisedTurretAngle);
            outtake.raisePrep();
        }

        // Outtake mid (dpad right)
        if (g2.dpadRightOnce()) {
            if (extendIntake) {
                intake.armIntake();
                intake.setExtenderMaxPower(0.6);
                if (intake.getExtenderTarget() < 50) intake.extenderTo(Intake.EXTENDER_BEFORE_CYCLE_TELEOP_POS);
                if (!intake.isClawOpen()) {
                    intake.clawRelease();
                    delayClawWide();
                }
            } else {
                if (intake.isArmBlockingOuttake()) intake.armStore();
            }

            if (outtake.getSlidePosition() < 50 && !outtake.isArmOut()) outtake.slideMid();
            else outtake.setSlidePosition(Outtake.SLIDE_MID + 50);

            raisedTurretAngle = 0;
            outtake.setTurretAngle(raisedTurretAngle);
            outtake.raisePrep();
        }

        // Outtake low (dpad down)
        if (g2.dpadDownOnce()) {
            if (intake.isArmBlockingOuttake()) intake.armStore();

            raisedTurretAngle = 0;
            outtake.setTurretAngle(raisedTurretAngle);
            outtake.scheduleLatch();
            outtake.armFlatOut();
            outtake.slideLow();
            outtake.guideRetractDown();
            //outtake.guideLow();

            if (!outtake.isSlideMagnetPresent()) scheduleSlideEncoderReset = true;
        }

        // Score and retract (A)
        if (g2.aOnce()) {
            outtake.latchOpen();
            outtake.guideRetractDown();
            if (!outtake.isSlideMagnetPresent()) scheduleSlideEncoderReset = true;
            new Thread(() -> { // TODO: Better wait solution
                sleep(150);
                outtake.store();
                outtake.turretCenter();
                outtake.guideStoreUp();
            }).start();
        }

        // Move guide and arm out (Y)
        if (g2.yOnce()) {
            outtake.raisePrep();
        }

        // Move guide and arm back (B)
        if (g2.bOnce()) {
            outtake.turretCenter();
            outtake.slideStore();
            outtake.guideRetractDown();
            outtake.armTransfer();
            if (!outtake.isSlideMagnetPresent()) scheduleSlideEncoderReset = true;
        }

        // Turret presets (Bumpers and X)
        if (g2.leftBumperOnce()) {
            raisedTurretAngle = Outtake.TURRET_LEFT;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        } else if (g2.rightBumperOnce()) {
            raisedTurretAngle = Outtake.TURRET_RIGHT;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        } else if (g2.xOnce()) {
            raisedTurretAngle = Outtake.TURRET_CENTER;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        }

        // Turret adjustment (Triggers)
        double triggers = g2.left_trigger - g2.right_trigger;
        if (Math.abs(triggers) > 0.01) {
            raisedTurretAngle = raisedTurretAngle + triggers*4;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        }

        // Slide adjustment (Left stick Y)
        double slideStick = -g2.left_stick_y;
        if (Math.abs(slideStick) < 0.01) { // if stick not engaged, use PID
            outtake.enableOuttakePID();
        } else { // if stick is engaged, apply manual power
            outtake.disableOuttakePID();
            outtake.setOuttakeOverridePower(Math.pow(slideStick, 3) * 0.75);
            raisedTurretAngle = outtake.getTurretAngle();
            if (outtake.getSlidePosition() < 80) { // center turret if too low
                outtake.turretCenter();
            }
            if (outtake.isSlideMagnetPresent()) {
                outtake.getSlide().zeroMotorInternals(); // directly reset
            }
        }

        // Arm adjustment (Right stick Y)
        double armStick = -g2.right_stick_y;
        if (Math.abs(armStick) > 0.01) {
            outtake.setArmPosition(outtake.getArmPosition() + armStick * -0.015);
        }

        if (scheduleSlideEncoderReset && outtake.isSlideDown()) {
            outtake.getSlide().zeroMotorInternals();
            scheduleSlideEncoderReset = false;
        }
    }

    private void delayClawWide() {
        new Thread(() -> { // TODO: Better wait solution
            sleep(100);
            if (intake.isArmOut() && !intake.isClawClosed()) { // requires that the intake is out and already open
                intake.clawWide();
            }
        }).start();
    }

    private double timeLeft() {
        if (!isStarted()) return 120;
        return 120 - smartGameTimer.seconds();
    }
    private boolean isBetween(double number, double min, double max) {
        return min <= number && number < max;
    }

    private void warnings() {
        // CAUTION: The rules prohibit flashing at a frequency faster than one blink per second
        if (outtake.isJunctionInGuide() && Math.abs(outtake.getGuidePosition() - Outtake.GUIDE_RETRACT_DOWN) > 0.01) { // If guide is up & junction in guide, display
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (isBetween(timeLeft(), 65, 75)) { // 75-70; start owning junctions
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            if (!warning1) {
                g1.rumbleBlips(3);
                warning1 = true;
            }
        } else if (isBetween(timeLeft(), 31, 35)) { // 35-31; prepare for endgame
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            if (!warning2) {
                g1.rumbleBlips(3);
                warning2 = true;
            }
        } else if (isBetween(timeLeft(), 25, 30)) { // 30-25; endgame starts
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            if (!warning3) {
                g1.rumbleBlips(3);
                warning3 = true;
            }
        } else if (isBetween(timeLeft(), 0, 5)) { // last 5 sec, go park
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            if (!warning4) {
                g1.rumbleBlips(3);
                warning4 = true;
            }
        } else { // default lights, put here for lower priority
            showGameLights();
        }
    }

    private void showGameLights() {
        // Custom light patterns when resetting an encoder, running automation, sensors activating, etc.
        // These are lower priority than the patterns in warnings()
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    private void autoStackControls() {
        g1.update();
        g2.update();
        // Turret
        double triggers = Range.clip((g2.left_trigger - g2.right_trigger) + (g1.left_trigger - g1.right_trigger), -1, 1);
        if (triggers != 0) {
            raisedTurretAngle = outtake.getTurretTarget() + triggers;
        }

        // Turret presets (Bumpers and X)
        if (g2.leftBumperOnce()) {
            raisedTurretAngle = Outtake.TURRET_LEFT;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        } else if (g2.rightBumperOnce()) {
            raisedTurretAngle = Outtake.TURRET_RIGHT;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        } else if (g2.xOnce()) {
            raisedTurretAngle = Outtake.TURRET_CENTER;
            if (outtake.getSlideTarget() > 50) outtake.setTurretAngle(raisedTurretAngle); // update turret
        }

        // Abort
        if (g1.xOnce() || !autoStack) {
            roadrunner.stopTrajectory();
            autoStack = false;
        }

        telemetry.addData("Turret Angle", outtake.getTurretAngle() + " -> " + outtake.getTurretTarget());
        telemetry.addData("Extender Pos", outtake.getSlidePosition() + " -> " + outtake.getSlidePosition());
        telemetry.update();
    }

    private void autoStackCycle() {
        Pose2d pose = roadrunner.getPoseEstimate();
        autoStack = true;

        intake.setExtenderMaxPower(1.0);

        roadrunner.followTrajectorySequence(auto.grabAndTransfer(builder(pose), stacknum));
        stacknum--;

        while (stacknum > 0 && autoStack) {
            roadrunner.followTrajectorySequence(auto.cycle(builder(pose), raisedTurretAngle, stacknum, null));
            if (!autoStack) return;
            stacknum--;
        }
        roadrunner.followTrajectorySequence(auto.sendLastCone(builder(pose), raisedTurretAngle));
    }

    private TrajectorySequenceBuilder builder(Pose2d pose) {
        return roadrunner.trajectorySequenceBuilder(pose)
                .addIterative("update", outtake::update)
                .addIterative("controls", this::autoStackControls);
    }
}
