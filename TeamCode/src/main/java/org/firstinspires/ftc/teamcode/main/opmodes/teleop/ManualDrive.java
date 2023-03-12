package org.firstinspires.ftc.teamcode.main.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.main.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.main.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.main.subsystems.Hub;
import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.Memory;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;

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

    public static boolean START_ON_INPUT = true;
    public static boolean DO_SENSOR_CHECK = true;
    public static boolean DO_AUTO_UP = true;


    private Drivetrain drivetrain;
    private Roadrunner roadrunner;
    private Hub hub;
    private Intake intake;
    private Outtake outtake;

    private GamePadController g1, g2;

    private boolean isSlow = false;

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
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            g1.update();
            g2.update();
            if (g1.x() || g2.x()) Memory.IS_BLUE = true;
            if (g1.b() || g2.b()) Memory.IS_BLUE = false;

            showGameLights();

            telemetry.addData("ALL Ready !!!", "Press START...");
            telemetry.addData("Color", Memory.IS_BLUE ? "🔵 BLUE" : "🔴 RED");
            telemetry.update();
        }

        // Transition, starts when g1 does an input. WARNING: May cause more user error than starting at the timer
        g1.update();
        while (START_ON_INPUT && g1.atRest() && opModeIsActive()) {
            g1.update();
            telemetry.addLine("Waiting for gamepad 1 input");
            telemetry.addData("Color", Memory.IS_BLUE ? "🔵 BLUE" : "🔴 RED");
            telemetry.update();
        };

        // On start (or when g1 did something)
        if (opModeIsActive()) {
            resetRuntime();
            g1.reset();
            g2.reset();
        }

        if (isStopRequested()) return; // exit if stopped

        // While running
        while (opModeIsActive()) {
            g1.update();
            g2.update();

            move();

            subsystemOne(); // example

            subsystemTwo(); // example

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
        telemetry.update();
    }

    private void move() {
        if (g1.xOnce()) isSlow = !isSlow;
        else if (g1.xLongOnce()) isSlow = false;

        double input_x = Math.pow(-g1.left_stick_y, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_y = Math.pow(-g1.left_stick_x, 3) * (g1.leftStickButton() ? 1 : (isSlow ? SLOW_SPEED : SPEED_CONSTANT));
        double input_turn = (g1.left_trigger - g1.right_trigger) * TURN_CONSTANT;

        if (g1.leftBumper()) input_turn += SLOW_TURN;
        if (g1.rightBumper()) input_turn -= SLOW_TURN;

        roadrunner.setDrivePower(new Pose2d(input_x, input_y, input_turn));
    }

    private void subsystemOne() { // TODO: Implement
        // E.g. intake related things
    }

    private void subsystemTwo() { // TODO: Implement
        // E.g. outtake related things
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
