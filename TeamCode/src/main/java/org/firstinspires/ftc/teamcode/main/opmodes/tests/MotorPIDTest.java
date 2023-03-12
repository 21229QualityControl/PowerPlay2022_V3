package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;

/**
 * This is a simple dash based test for motors under our PID controller.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class MotorPIDTest extends LinearOpMode {
    public static String NAME = "outtakeMotor";
    public static boolean REVERSED = true;
    public static int POSITION = 0;
    public static double MAX_POWER = 0.01;
    public static PIDCoefficients pid = new PIDCoefficients(0.013, 0, 0.0001);
    public static int FEEDFORWARD_MIN = 300;
    public static double FEEDFORWARD = 0.05;

    public static boolean RESET_ENCODERS = false;

    private MotorWithPID motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, NAME), pid, (x, v) -> (x > FEEDFORWARD_MIN ? FEEDFORWARD : 0.0));

        if (RESET_ENCODERS) {
            motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (REVERSED) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else motor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Instructions", "This program is dependent on the dashboard and its variable config");
        telemetry.addData("Motor name", NAME);
        telemetry.addData("Motor reversed", REVERSED);
        telemetry.addData("Encoders reset", RESET_ENCODERS);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor.setTargetPosition(POSITION);
            motor.setMaxPower(MAX_POWER);

            motor.update();

            telemetry.addData("Target Position", motor.getTargetPosition());
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Error", motor.getTargetPosition() - motor.getCurrentPosition());
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}
