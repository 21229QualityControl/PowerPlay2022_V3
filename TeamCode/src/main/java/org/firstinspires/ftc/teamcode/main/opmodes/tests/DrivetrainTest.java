package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.subsystems.GamePadController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;

import java.util.Arrays;
import java.util.List;

/**
 * This teleop tests the consistency of the four drivetrain motors
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class DrivetrainTest extends LinearOpMode {
    public static double POWER = 0;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create our drivetrain motors
        leftFront = HardwareCreator.createMotor(hardwareMap, "leftFront");
        leftRear = HardwareCreator.createMotor(hardwareMap, "leftRear");
        rightRear = HardwareCreator.createMotor(hardwareMap, "rightRear");
        rightFront = HardwareCreator.createMotor(hardwareMap, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setMaxRPM(DriveConstants.MAX_RPM);
            motorConfigurationType.setTicksPerRev(DriveConstants.TICKS_PER_REV / DriveConstants.GEAR_RATIO);
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        // reset power to zero
        POWER = 0;

        waitForStart();

        // Exit immediately if stopped
        if (isStopRequested()) return;

        // Game loop
        while (!isStopRequested()) {
            // Update controller
            g1.update();

            double setPower;

            if (g1.dpadUpOnce()) {
                POWER = Range.clip(POWER + 0.01, -1, 1);
            }
            if (g1.dpadDownOnce()) {
                POWER = Range.clip(POWER - 0.01, -1, 1);
            }
            if (g1.dpadRightOnce()) {
                POWER = Range.clip(POWER + 0.1, -1, 1);
            }
            if (g1.dpadLeftOnce()) {
                POWER = Range.clip(POWER - 0.1, -1, 1);
            }
            if (g1.bOnce()) POWER = 0;

            if (g1.a()) {
                setPower = -Math.pow(g1.left_stick_y, 3);
            } else {
                setPower = POWER;
            }

            for (DcMotorEx motor : motors) {
                motor.setPower(setPower);
            }

            // Telemetry state and instructions
            telemetry.addLine("~ Use dpad to step motor powers");
            telemetry.addLine("~ Use B to stop the motors");
            telemetry.addLine("~ Hold A and use left stick to manually control power");
            telemetry.addLine();
            telemetry.addData("Sent Power", setPower);
            telemetry.addData("Saved Power", POWER);
            telemetryMotor("Left front", leftFront);
            telemetryMotor("Left rear", leftRear);
            telemetryMotor("Right rear", rightRear);
            telemetryMotor("Right front", rightFront);
            telemetry.update();
        }
    }

    private void telemetryMotor(String name, DcMotorEx motor) {
        telemetry.addData(name, "%d (%.2f °/s)", motor.getCurrentPosition(), motor.getVelocity(AngleUnit.DEGREES));
    }
}