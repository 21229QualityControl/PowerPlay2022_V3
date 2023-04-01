package org.firstinspires.ftc.teamcode.main.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;

/**
 * This is a simple dash based test for all of the RunModes on a motor.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class MotorTest extends LinearOpMode {
    public static String NAME = "motor";
    public static boolean REVERSED = false;
    public static boolean BRAKE = false;
    public static Mode MODE = Mode.Power;
    public static double VELOCITY = 0;
    public static int POSITION = 0;
    public static double POWER = 0;
//    public static MotorPIDValues PID = new MotorPIDValues();

    public enum Mode {
        Power,
        Velocity,
        Position
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = HardwareCreator.createMotor(hardwareMap, NAME);
        motor.setPower(0);
        motor.setTargetPosition(0);

        telemetry.addData("Instructions", "This program is dependent on the dashboard and its variable config");
        telemetry.addData("Motor name", NAME);
        telemetry.addData("Motor reversed", REVERSED);
        telemetry.update();

        waitForStart();

        telemetry.addData("Running", "Check the dashboard");
        telemetry.update();

        while (!isStopRequested()) {
            if (BRAKE) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            switch (MODE) {
                case Power:
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(POWER);

                    telemetry.addData("-Mode", "RUN_WITHOUT_ENCODER (TO_POWER)");
                    telemetry.addData("Power", motor.getPower());

                    telemetry.addData("-Position", motor.getCurrentPosition());
                    telemetry.addData("-Velocity", motor.getVelocity());
                    break;
                case Velocity:
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocity(VELOCITY, AngleUnit.DEGREES);

                    telemetry.addData("-Mode", "RUN_USING_ENCODER (TO_VELOCITY)");
                    telemetry.addData("Velocity", motor.getVelocity());
                    telemetry.addData("Velocity Target", VELOCITY);

                    telemetry.addData("-Position", motor.getCurrentPosition());
                    break;
                case Position:
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setTargetPosition(POSITION);
                    motor.setPower(POWER);

                    telemetry.addData("-Mode", "RUN_TO_POSITION (TO_POSITION)");
                    telemetry.addData("Position", motor.getCurrentPosition());
                    telemetry.addData("Position Target", motor.getTargetPosition());
                    telemetry.addData("Max Power", motor.getPower());

                    telemetry.addData("-Velocity", motor.getVelocity());
                    break;
            }

//            motor.setPositionPIDFCoefficients(PID.POS_P);
//            motor.setVelocityPIDFCoefficients(PID.VEL_P, PID.VEL_I, PID.VEL_D, PID.VEL_F);

            telemetry.update();
        }
    }

//    public static class MotorPIDValues {
//        public double POS_P = 0; // don't use
//        public double VEL_P = 12;
//        public double VEL_I = 0.9;
//        public double VEL_D = 0;
//        public double VEL_F = 0;
//    }
}
