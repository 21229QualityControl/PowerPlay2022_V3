package org.firstinspires.ftc.teamcode.main.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_HEADING_PID;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner.KEEP_POSITION_TRANSLATIONAL_PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import java.util.List;

/**
 * Main Driver program for teleop
 *
 * The controls are below
 *
 * Player 1:
 * - Strafe (Left Stick)
 * - Full speed (hold Left Stick Button)
 * - Slow speed toggle (X)
 * - Turns (Bumpers and Triggers)
 *
 * Player 2:
 * - N/A
 */
@Config
@TeleOp(group = "Drive", name = "Manual Drive")
public class ManualDrive extends LinearOpMode {
    public static double SLOW_SPEED = 0.4;
    public static double SLOW_TURN = 0.2;
    public static double SPEED_CONSTANT = 0.8;
    public static double TURN_CONSTANT = 0.9;

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
    private boolean slideExtend = false;
    private boolean armOut = false;

    private boolean warning1 = false;
    private boolean warning2 = false;
    private boolean warning3 = false;

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

            smartGameTimer = new SmartGameTimer(false);
        }

        positionMaintainer = new PositionMaintainer(KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_HEADING_PID, KEEP_POSITION_TOLERANCE.asPose2d());
        led = new LED(hardwareMap);
        auto = new Cycler(roadrunner, intake, outtake);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.initialize();
        outtake.initialize();
        outtake.setTurretAngle(0);

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

            outtake.initialize();
            intake.initialize();
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
        Memory.LAST_POSE = roadrunner.getPoseEstimate();
    }

    private void sendTelemetry() {
        List<Double> vels = drivetrain.getMotorPowers();
        Dashboard.packet.put("Battery Voltage", hub.getVoltage());
        Dashboard.packet.put("Runtime", getRuntime());
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Pos", roadrunner.getPoseEstimate());
        telemetry.addData("Vel", roadrunner.getPoseVelocity());
        telemetry.addData("Power", "\n   %4.0f%% | %4.0f%% \n   %4.0f%% | %4.0f%%", vels.get(0)*100, vels.get(3)*100, vels.get(1)*100, vels.get(2)*100);

        telemetry.addData("Debug: Arm Out", armOut);
        telemetry.update();
    }

    private void move() {
        if (g1.leftStickButtonOnce()) isSlow = !isSlow;
        else if (g1.leftStickButtonLong()) isSlow = false;

        double input_x = Math.pow(-g1.left_stick_y, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_y = Math.pow(-g1.left_stick_x, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_turn = (g1.left_trigger - g1.right_trigger) * TURN_CONSTANT;

        if (g1.leftBumper()) input_turn += SLOW_TURN;
        if (g1.rightBumper()) input_turn -= SLOW_TURN;

        roadrunner.setDrivePower(new Pose2d(input_x, input_y, input_turn));
    }

    private void intakeControls() {
        if (g1.startOnce()) { // Toggle extension setting
            if (slideExtend) { // turn it off
                intake.extendStore();
                slideExtend = false;
            } else { // turn it on
                intake.extendCycle();
                slideExtend = true;
            }
        }

        if (g1.xOnce()) {
            if (armOut) { // X to bring in arm and claw
                intake.extendStore();
                intake.armStore();
                intake.clawGrab();
                intake.vslideDown();
                armOut = false;
            } else { // X to put out arm and claw
                if (slideExtend) intake.extendCycle();
                intake.armIntake();
                intake.clawWide();
                intake.vslideLevel(1);
                armOut = true;
            }
        }
        if (g1.yOnce()) { // Y to transfer
            intake.extendStore();
            intake.clawGrab();
            intake.extendStore();
            outtake.turretCenter();
            outtake.armTransfer();
            intake.vslideTransfer();

            new Thread(() -> { // TODO: Better wait solution
                sleep(500);
                intake.armTransfer();
                sleep(500);
                intake.clawWide();
                sleep(500);
                intake.vslideDown();
                intake.armStore();
                intake.clawGrab();
                armOut = false;
            }).start();
        }
        if (g1.aOnce()) { // A to grab
            intake.clawGrab();
        }
        if (g1.bOnce()) { // B to let go
            if (intake.isArmFlat() || (intake.isArmOut() && !intake.isClawClosed()) || (intake.isArmBlockingOuttake())) intake.clawWide();
            else intake.clawRelease();
        }
    }

    enum TURRET_POS {
        LEFT,
        CENTER,
        RIGHT,
    }
    public static TURRET_POS turretPos = TURRET_POS.CENTER;

    private void moveTurret() {
        switch (turretPos) {
            case LEFT:
                outtake.turretLeft();
                break;

            case RIGHT:
                outtake.turretRight();
                break;

            case CENTER:
                outtake.turretCenter();
                break;
        }
    }

    private void outtakeControls() {
        if (g2.dpadUpOnce()) { // Dpad up to raise high
            intake.armStore();
            intake.vslideDown();
            armOut = false;

            moveTurret();
            outtake.raiseHigh();
        }
        if (g2.dpadRightOnce()) { // Dpad right to raise mid
            intake.armStore();
            intake.vslideDown();
            armOut = false;

            moveTurret();
            outtake.raiseMid();
        }
        if (g2.dpadDownOnce()) { // Dpad down to raise low
            intake.armStore();
            intake.vslideDown();
            armOut = false;

            moveTurret();
            outtake.raiseLow();
        }
        if (g2.dpadLeftOnce()) { // Dpad left to slightly lower slide (good for aiming)
            intake.armStore();
            intake.vslideDown();
            armOut = false;

            outtake.setSlidePosition(outtake.getSlidePosition() - 100);
        }
        if (g2.bOnce()) { // B to score
            outtake.latchOpen();
            outtake.guideRetractDown();
            new Thread(() -> { // TODO: Better wait solution
                sleep(150);
                outtake.store();
                outtake.turretCenter();
                outtake.guideStoreUp();
            }).start();
        }
        if (g2.rightBumperOnce()) { // Right bumper to point turret right
            turretPos = TURRET_POS.RIGHT;
        }
        if (g2.leftBumperOnce()) { // Left bumper to point turret left
            turretPos = TURRET_POS.LEFT;
        }
        if (g2.xOnce()) { // X to center turret
            turretPos = TURRET_POS.CENTER;
        }
    }

    private double timeLeft() {
        if (!isStarted()) return 120;
        return 120 - getRuntime();
    }
    private boolean isBetween(double number, double min, double max) {
        return min <= number && number < max;
    }

    private void warnings() {
        if (isBetween(timeLeft(), 0, 5)) { // last 5 sec, go park
            if (!warning3) {
                g1.rumbleBlips(1);
                warning3 = true;
            }
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED); // WARNING: The rules prohibit flashing at a frequency faster than one blink per second
        } else if (isBetween(timeLeft(), 27, 30)) { // 30-29; endgame starts
            if (!warning2) {
                g1.rumbleBlips(2);
                warning2 = true;
            }
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        } else if (isBetween(timeLeft(), 40, 45)) { // 45-40; start capping
            if (!warning1) {
                g1.rumbleBlips(3);
                warning1 = true;
            }
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        } else { // default lights, put here for lower priority
            showGameLights();
        }
    }

    private void showGameLights() { // TODO: Implement
        // Custom light patterns when resetting an encoder, running automation, sensors activating, etc.
    }
}
