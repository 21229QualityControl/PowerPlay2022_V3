package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.hardware.fakes.CRServoFake;
import org.firstinspires.ftc.teamcode.util.hardware.fakes.DcMotorFake;
import org.firstinspires.ftc.teamcode.util.hardware.fakes.ServoFake;


/**
 * Class that creates most of our hardware objects
 * Intercepts simulation and substitutes in a fake hardware device
 * Intercepts deviceNotFound and gives warning and a fake device
 */
@Config
public class HardwareCreator {
    public static boolean SIMULATE_HARDWARE = false;
    public static boolean SIMULATE_DRIVETRAIN = false;

    public enum ServoType {
        DEFAULT(600, 2400),
        GOBILDA(500, 2500),
        AXON(500, 2500);

        public final PwmControl.PwmRange pwmRange;

        ServoType(double usPulseLower, double usPulseUpper) {
            this.pwmRange = new PwmControl.PwmRange(usPulseLower, usPulseUpper);
        }
    }

    public static DcMotorEx createMotor(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new DcMotorFake();
        try {
            return hardwareMap.get(DcMotorEx.class, deviceName);
        } catch (IllegalArgumentException e) { // Could not find device
            RobotLog.addGlobalWarningMessage("Failed to find DcMotorEx '%s'", deviceName);
            return new DcMotorFake();
        }
    }

    public static Servo createServo(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new ServoFake();
        try {
            return hardwareMap.get(ServoImplEx.class, deviceName);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find Servo '%s'", deviceName);
            return new ServoFake();
        }
    }

    public static CRServo createCRServo(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new CRServoFake();
        try {
            return hardwareMap.get(CRServo.class, deviceName);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find CRServo '%s'", deviceName);
            return new CRServoFake();
        }
    }

    public static void setServoRange(Servo servo, ServoType servoType) {
        ((PwmControl) servo).setPwmRange(servoType.pwmRange);
    }
}
