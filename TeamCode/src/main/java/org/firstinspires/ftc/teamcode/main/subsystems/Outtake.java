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
    private static int TURRET_MIN = 0;
    private static int TURRET_MAX = 0;
    public static int TURRET_CENTER = 0; // Initialize with turret centered
    public static int TURRET_LEFT = 300; // TODO: Tune
    public static int TURRET_RIGHT = -300; // TODO: Tune

    public static PIDCoefficients SLIDE_PID = new PIDCoefficients(0.015, 0, 0.0004);
    public static int SLIDE_HIGH = 600; // 900 is highest position
    public static int SLIDE_MID = 270;
    public static int SLIDE_LOW = 100;
    public static int SLIDE_STORED = 0;
    public static int SLIDE_OFFSET = 100;

    private static double ARM_MIN = 0;
    private static double ARM_MAX = 0.85;
    public static double ARM_FLAT_OUT = 0.11;
    public static double ARM_TILT_OUT = 0.17;
    public static double ARM_VERTICAL = 0.38; // might be unnecessary
    public static double ARM_TRANSFER = 0.80; // TODO: Tune

    public static double LATCH_OPEN = 0.37;
    public static double LATCH_BARELY = 0.74; // Just covers the bottom, does not apply pressure
    public static double LATCH_ENGAGED = 0.84; // may disable the servo temporarily

    private static double GUIDE_MIN = 0;
    private static double GUIDE_MAX = 0;
    public static double GUIDE_FLAT = 0.4;
    public static double GUIDE_INIT = 0.75;
    public static double GUIDE_RETRACT_DOWN = 0.75;
    public static double GUIDE_STORE_UP = 0.05;

    public Outtake(HardwareMap hardwareMap) {
        this.turret = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeTurret"), TURRET_PID);
        this.slide = new OppositeMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide1"), HardwareCreator.createMotor(hardwareMap, "outtakeSlide2"), SLIDE_PID);
        this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch", HardwareCreator.ServoType.DEFAULT);
        this.guide = HardwareCreator.createServo(hardwareMap, "outtakeGuide", HardwareCreator.ServoType.GOBILDA);
        this.arm = HardwareCreator.createServo(hardwareMap, "outtakeArm", HardwareCreator.ServoType.AXON);

        this.slide.setMaxPower(0.6);
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
        this.arm.setPosition(ARM_TILT_OUT);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER);
    }

    // Latch movements
    public void latchOpen() {
        this.latch.setPosition(LATCH_OPEN);
    }
    public void latchClosed() {
        this.latch.setPosition(LATCH_BARELY);
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
        guide.setPosition(GUIDE_STORE_UP);
    }
    public void guideOut() {
        guide.setPosition(GUIDE_FLAT);
    }
    // Only needed for initialization stuff
    public void guideDown() {
        guide.setPosition(GUIDE_RETRACT_DOWN);
    }

    // Shortcuts
    public void store() {
        armTransfer();
        slideStore();
        guideIn();
        latchOpen();
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
