package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;

/**
 * This is a simple dash based test for servo positions.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class DualServoTest extends LinearOpMode {
    public static ServoSettings SERVO_1 = new ServoSettings("Servo 1");
    public static ServoSettings SERVO_2 = new ServoSettings("Servo 2");
    public static double POSITION_1 = 0.5;
    public static double POSITION_2 = 0.5;
    public static boolean SYNC_POS2_WITH_POS1 = false;
    public static boolean UPDATE_RANGE = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo1 = SERVO_1.generateServo(hardwareMap);
        Servo servo2 = SERVO_2.generateServo(hardwareMap);

        telemetry.addData("Instructions", "This program is dependent on the dashboard and its variable config");
        telemetry.addData("Servo 1 name", SERVO_1.NAME);
        telemetry.addData("Servo 2 name", SERVO_2.NAME);
        telemetry.update();

        waitForStart();

        telemetry.addData("Running", "Check the dashboard");
        telemetry.update();

        while (!isStopRequested()) {

            if (UPDATE_RANGE) {
                servo1.scaleRange(Math.min(SERVO_1.MIN, SERVO_1.MAX), Math.max(SERVO_1.MIN, SERVO_1.MAX));
                servo2.scaleRange(Math.min(SERVO_2.MIN, SERVO_2.MAX), Math.max(SERVO_2.MIN, SERVO_2.MAX));
                UPDATE_RANGE = false;
            }

            if (SYNC_POS2_WITH_POS1) {
                POSITION_2 = POSITION_1;
            }

            servo1.setPosition(POSITION_1);
            servo2.setPosition(SYNC_POS2_WITH_POS1 ? POSITION_1 : POSITION_2);

            telemetry.addData("Position 1", servo1.getPosition());
            telemetry.addData("Position 2", servo2.getPosition());
            telemetry.update();
        }
    }

    public static class ServoSettings {
        public String NAME;
        public boolean REVERSED;
        public HardwareCreator.ServoType TYPE;
        public double MIN = 0;
        public double MAX = 1;

        public ServoSettings(String name, boolean reversed, HardwareCreator.ServoType type, double min, double max) {
            this.NAME = name;
            this.REVERSED = reversed;
            this.TYPE = type;
            this.MIN = min;
            this.MAX = max;
        }

        public ServoSettings(String name) {
            this(name, false, HardwareCreator.ServoType.DEFAULT, 0, 1);
        }

        public Servo generateServo(HardwareMap hardwareMap) {
            Servo servo = HardwareCreator.createServo(hardwareMap, NAME);

            if (REVERSED) servo.setDirection(Servo.Direction.REVERSE);
            else servo.setDirection(Servo.Direction.FORWARD);

            HardwareCreator.setServoRange(servo, TYPE);

            servo.scaleRange(Math.min(MIN, MAX), Math.max(MIN, MAX));

            return servo;
        }
    }
}
