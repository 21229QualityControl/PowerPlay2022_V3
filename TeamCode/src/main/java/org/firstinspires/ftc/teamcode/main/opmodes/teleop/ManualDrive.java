package org.firstinspires.ftc.teamcode.main.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_HEADING_PID;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TRANSLATIONAL_PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.Cycler;
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
    public static double SLOW_TURN = 0.2;
    public static double SPEED_CONSTANT = 0.9;
    public static double TURN_CONSTANT = 0.75;
    public static double AUTO_TRANSFER_COOLDOWN = 200; // ms

    private Drivetrain drivetrain;
    private Roadrunner roadrunner;
    private Intake intake;
    private Outtake outtake;
    private Hub hub;
    private Cycler auto;
    private LED led;

    private PositionMaintainer positionMaintainer;
    private ElapsedTime autoTransferTimer;
    private SmartGameTimer smartGameTimer;

    private GamePadController g1, g2;

    private boolean isSlow = false;
    private boolean keepPosition = false;
    private boolean extendIntake = false;
    private boolean autoStack = false;
    private boolean autoTransfer = false;
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
        intake = new Intake(hardwareMap);
        if (Memory.REMEMBERED_OUTTAKE != null) {
            outtake = new Outtake(hardwareMap, Memory.REMEMBERED_OUTTAKE);
            smartGameTimer = new SmartGameTimer(true);
        } else { // WE DON'T HAVE MEMORY OF AUTO!!! we need to align the outtake motors
            outtake = new Outtake(hardwareMap);
            outtake.getSlide().getMainMotor().setPower(-0.3);
            outtake.getSlide().getSecondMotor().setPower(-0.3);
            intake.getExtender().getMotor().setPower(-0.3);

            smartGameTimer = new SmartGameTimer(false);
        }

        positionMaintainer = new PositionMaintainer(KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_HEADING_PID, KEEP_POSITION_TOLERANCE.asPose2d());
        led = new LED(hardwareMap);
        auto = new Cycler(roadrunner, intake, outtake);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.initialize();
        outtake.initialize();
        outtake.setTurretAngle(0);

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
            }
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
        telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
        telemetry.addData("Turret Angle", outtake.getTurretAngle() + " -> " + outtake.getTurretTarget());
        telemetry.addData("Outtake Pos", outtake.getSlidePosition() + " -> " + outtake.getSlideTarget());
        telemetry.addData("Extender Pos", intake.getExtenderPosition() + " -> " + intake.getExtenderTarget());
        telemetry.addData("Power", "\n   %4.0f%% | %4.0f%% \n   %4.0f%% | %4.0f%%", vels.get(0)*100, vels.get(3)*100, vels.get(1)*100, vels.get(2)*100);
        telemetry.update();
    }

    private void move() {
        // Toggle slow mode, or turn off keepPosition if it is on
        if (g1.leftStickButtonOnce()) {
            if (keepPosition) keepPosition = false;
            else isSlow = !isSlow;
        }

        // Toggle keep position mode
        if (g1.backOnce()) {
            keepPosition = !keepPosition;
            positionMaintainer.resetController();
            positionMaintainer.maintainPosition(roadrunner.getPoseEstimate());
        }

        // Movement inputs
        double input_x = Math.pow(-g1.left_stick_y, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_y = Math.pow(-g1.left_stick_x, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_turn = (g1.left_trigger - g1.right_trigger) * TURN_CONSTANT;

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
            roadrunner.setDrivePower(new Pose2d(input_x, input_y, input_turn));
        }
    }

    private void intakeControls() {
        // Open/Close Claw (A/B)
        if (g1.aOnce()) { // grab
            intake.clawGrab();
        }
        if (g1.bOnce()) { // release
            if (intake.isArmFlat() || (intake.isArmOut() && !intake.isClawClosed())) intake.clawWide();
            else intake.clawRelease();
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
            intake.extendStore();
            intake.clawGrab();
            intake.extendStore();
            outtake.turretCenter();
            outtake.armTransfer();
            intake.vslideTransfer();

            new Thread(() -> { // TODO: Better wait solution
                sleep(250);
                intake.armTransfer();
                sleep(400);
                intake.clawRelease();
                sleep(300);
                intake.vslideDown();
                intake.armStore();
            }).start();
        }

        // Go Storage or Intake (X)
        if (g1.xOnce()) {
            if (intake.isArmOut()) { // Go Storage
                intake.extendStore();
                intake.armStore();
                intake.vslideDown();
                if (intake.isClawOpen()) intake.clawRelease();
            } else { // Go Intake
                if (extendIntake) intake.extendCycleTeleop();
                intake.armIntake();
                intake.vslideLevel(1);
                intake.clawRelease();
                delayClawWide();
//                autoTransferTimer.reset();
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
            if (extendIntake) intake.extendCycleTeleop();
            else intake.extendStore();
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
                intake.clawRelease();
                delayClawWide();
                intake.extendCycleTeleop();
                intake.armIntake();
            }
            intake.vslideLevel(stacknum);
            autoTransferTimer.reset();
        }

        // Auto stack cycle
        if (g1.aLongOnce()) {
            autoStackCycle();
        }
    }

    private void outtakeControls() {
        // Outtake high (dpad up)
        if (g2.dpadUpOnce()) {
            if (intake.isArmBlockingOuttake()) intake.armStore();

            outtake.setTurretAngle(raisedTurretAngle);
            outtake.raisePrep();
            if (outtake.getSlidePosition() < 50) outtake.slideHigh();
            else outtake.setSlidePosition(Outtake.SLIDE_HIGH + 50);
        }

        // Outtake mid (dpad right)
        if (g2.dpadRightOnce()) {
            if (intake.isArmBlockingOuttake()) intake.armStore();

            raisedTurretAngle = 0;
            outtake.setTurretAngle(raisedTurretAngle);
            outtake.raisePrep();
            if (outtake.getSlidePosition() < 50) outtake.slideMid();
            else outtake.setSlidePosition(Outtake.SLIDE_MID + 50);
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
        }

        // Score and retract (A)
        if (g2.aOnce()) {
            outtake.latchOpen();
            outtake.guideRetractDown();
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
            outtake.guideRetractDown();
            outtake.armTransfer();
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
        } else if (slideStick < 0 && outtake.getSlidePosition() <= 10) { // if forcing down, target bottom
            outtake.enableOuttakePID();
            outtake.turretCenter(); // center turret when too low
        } else { // if stick is engaged and not forcing down, apply manual power
            outtake.disableOuttakePID();
            outtake.setOuttakeOverridePower(Math.pow(slideStick, 3) * 0.75);
            raisedTurretAngle = outtake.getTurretAngle();
            if (outtake.getSlidePosition() < 80) { // center turret if too low
                outtake.turretCenter();
            }
        }

        // Arm adjustment (Right stick Y)
        double armStick = -g2.right_stick_y;
        if (Math.abs(armStick) > 0.01) {
            outtake.setArmPosition(outtake.getArmPosition() + armStick * -0.015);
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
        if (isBetween(timeLeft(), 65, 75)) { // 75-70; start owning junctions
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

    private void showGameLights() { // TODO: Implement
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
