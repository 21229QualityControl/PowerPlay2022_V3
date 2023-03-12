package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;

/**
 * This is a simple dash based test for servo positions.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class ServoTest extends LinearOpMode {
    public static String NAME = "armServo";
    public static boolean REVERSED = false;
    public static HardwareCreator.ServoType SERVO_TYPE = HardwareCreator.ServoType.DEFAULT;
    public static boolean CHANGE_RANGE = false;
    public static double MIN = 0;
    public static double MAX = 1;
    public static double POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo = HardwareCreator.createServo(hardwareMap, NAME);

        if (REVERSED) servo.setDirection(Servo.Direction.REVERSE);
        else servo.setDirection(Servo.Direction.FORWARD);

        HardwareCreator.setServoRange(servo, SERVO_TYPE);

        telemetry.addData("Instructions", "This program is dependent on the dashboard and its variable config");
        telemetry.addData("Servo name", NAME);
        telemetry.addData("Servo type", SERVO_TYPE);
        telemetry.addData("Servo reversed", REVERSED);

        telemetry.update();

        waitForStart();

        telemetry.addData("Running", "Check the dashboard");
        telemetry.update();

        while (!isStopRequested()) {

            if (CHANGE_RANGE) {
                servo.scaleRange(Math.min(MIN, MAX), Math.max(MIN, MAX));
                CHANGE_RANGE = false;
            }

            servo.setPosition(POSITION);

            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
