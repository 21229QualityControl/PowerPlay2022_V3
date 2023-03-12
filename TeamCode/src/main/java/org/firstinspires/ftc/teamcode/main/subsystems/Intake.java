package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;

@Config
public class Intake {
    private MotorWithPID extender;
    private Servo arm;
    private Servo claw;
    private Servo slide;

    public static PIDCoefficients EXTENDER_PID;
    public static int EXTENDER_IN = 0;
    public static int EXTENDER_OUT = 0;

    public static double ARM_TRANSFER = 0;
    public static double ARM_OUT = 0;

    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSED = 0;

    public static double SLIDE_UP = 0;
    public static double SLIDE_DOWN = 0;

    public Intake(HardwareMap hardwareMap) {
        this.extender = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "extender"), EXTENDER_PID);
        this.arm = HardwareCreator.createServo(hardwareMap, "wrist", HardwareCreator.ServoType.AXON);
        this.claw = HardwareCreator.createServo(hardwareMap, "claw", HardwareCreator.ServoType.AXON);
        this.slide = HardwareCreator.createServo(hardwareMap, "intakeSlide", HardwareCreator.ServoType.AXON);
    }

    public void initialize() {
        extendIn();
        armTransfer();
        clawClosed();
        slideDown();
    }

    public void update() {
        extender.update();
    }

    // Extender
    public void extendIn() {
        this.extender.setTargetPosition(EXTENDER_IN);
    }
    public void extendOut() {
        this.extender.setTargetPosition(EXTENDER_IN);
    }

    // Arm
    public void armOut() {
        this.arm.setPosition(ARM_OUT);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER);
    }

    // Claw
    public void clawOpen() {
        this.claw.setPosition(CLAW_OPEN);
    }
    public void clawClosed() {
        this.claw.setPosition(CLAW_CLOSED);
    }

    // Slide
    public void slideUp() {
        this.slide.setPosition(SLIDE_UP);
    }
    public void slideDown() {
        this.slide.setPosition(SLIDE_DOWN);
    }
}
