package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;

@Config
public class Outtake {
    private MotorWithPID slide;
    private MotorWithPID turret;
    private Servo latch;
    private Servo arm;

    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.007, 0, 0.0003);
    public static int TURRET_LEFT = 0;
    public static int TURRET_RIGHT = 0;
    public static int TURRET_CENTER = 0;

    public static PIDCoefficients SLIDE_PID = new PIDCoefficients(0.015, 0, 0.0004);
    public static int SLIDE_HIGH = 0;
    public static int SLIDE_MID = 0;
    public static int SLIDE_LOW = 0;
    public static int SLIDE_STORED = 0;
    public static int SLIDE_OFFSET = 0;

    public static double ARM_OUT;
    public static double ARM_TRANSFER;

    public static double LATCH_OPEN;
    public static double LATCH_CLOSED;

    public Outtake(HardwareMap hardwareMap) {
        this.turret = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "turret"), TURRET_PID);
        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), SLIDE_PID);
        this.latch = HardwareCreator.createServo(hardwareMap, "latch", HardwareCreator.ServoType.DEFAULT);
        this.arm = HardwareCreator.createServo(hardwareMap, "outtakeArm", HardwareCreator.ServoType.AXON);
    }

    public void initialize() {
        store();
        turretCenter();
    }

    public void update() {
        turret.update();
        slide.update();
    }

    // Arm movements
    public void armOut() {
        this.arm.setPosition(ARM_OUT);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER);
    }

    // Latch movements
    public void latchOpen() {
        this.latch.setPosition(LATCH_OPEN);
    }
    public void latchClosed() {
        this.latch.setPosition(LATCH_CLOSED);
    }

    // Slide movements
    public void slideStore() {
        slide.setTargetPosition(SLIDE_STORED);
    }
    public void slideLow() {
        slide.setTargetPosition(SLIDE_LOW);
    }
    public void slideMid() {
        slide.setTargetPosition(SLIDE_MID);
    }
    public void slideHigh() {
        slide.setTargetPosition(SLIDE_HIGH);
    }
    public void offsetSlide() {
        slide.setTargetPosition(slide.getTargetPosition() - SLIDE_OFFSET);
    }

    // Turret movements
    public void turretLeft() {
        turret.setTargetPosition(TURRET_LEFT);
    }
    public void turretCenter() {
        turret.setTargetPosition(TURRET_CENTER);
    }
    public void turretRight() {
        turret.setTargetPosition(TURRET_RIGHT);
    }

    // Shortcuts
    public void store() {
        armTransfer();
        slideStore();
    }
    private void raisePrep() {
        latchClosed();
        armOut();
    }
    public void raiseLow() {
        raisePrep();
        slideLow();
    }
    public void raiseMid() {
        raisePrep();
        slideMid();
    }
    public void raiseHigh() {
        raisePrep();
        slideHigh();
    }
}
