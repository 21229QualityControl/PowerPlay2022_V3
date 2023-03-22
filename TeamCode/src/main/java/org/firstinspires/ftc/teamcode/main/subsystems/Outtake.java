package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.hardware.OppositeMotorWithPID;

@Config
public class Outtake {
    private OppositeMotorWithPID slide;
    private MotorWithPID turret;
    private Servo latch;
    private Servo arm;
    private Servo guide;

    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.007, 0, 0.0003);
    public static int TURRET_LEFT = 300;
    public static int TURRET_RIGHT = -300;
    public static int TURRET_CENTER = 0; // Initialize with turret centered

    public static PIDCoefficients SLIDE_PID = new PIDCoefficients(0.015, 0, 0.0004);
    public static int SLIDE_HIGH = 920; // 920 is highest position
    public static int SLIDE_MID = 400;
    public static int SLIDE_LOW = 100;
    public static int SLIDE_STORED = 0;
    public static int SLIDE_OFFSET = 0;

    public static double ARM_OUT = 0; // This is actually 0
    public static double ARM_TRANSFER = 0.55;

    public static double LATCH_OPEN = 0.5;
    public static double LATCH_CLOSED = 0.84;

    public static double GUIDE_OUT = 0.4;
    public static double GUIDE_IN = 0.7;

    public Outtake(HardwareMap hardwareMap) {
        this.turret = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "turret"), TURRET_PID);
        this.slide = new OppositeMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), HardwareCreator.createMotor(hardwareMap, "outtakeOpposite"), SLIDE_PID);
        this.latch = HardwareCreator.createServo(hardwareMap, "latch", HardwareCreator.ServoType.DEFAULT);
        this.guide = HardwareCreator.createServo(hardwareMap, "guide", HardwareCreator.ServoType.GOBILDA);
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

    // Guide
    public void guideIn() {
        guide.setPosition(GUIDE_IN);
    }
    public void guideOut() {
        guide.setPosition(GUIDE_OUT);
    }

    // Shortcuts
    public void store() {
        armTransfer();
        slideStore();
        guideIn();
    }
    private void raisePrep() {
        latchClosed();
        armOut();
        guideOut();
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
