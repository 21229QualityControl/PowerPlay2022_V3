package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(0.013, 0, 0.0001);
    private static int EXTENDER_MIN = 0;
    private static int EXTENDER_MAX = 1050; // 1080 is technically the max
    public static int EXTENDER_STORED_POS = 0;
    public static int EXTENDER_CYCLE_POS = 200; // TODO: Tune
    public static int EXTENDER_TRANSFER_POS = 0; // TODO: Tune

    private static double ARM_MIN = 0;
    private static double ARM_MAX = 0.87;
    public static double ARM_STORED_POS = 0.29; // Arm is placed vertical
    public static double ARM_INTAKING_POS = 0.6; // Claw leveled
    public static double ARM_TRANSFER_POS = 0.23;
    public static double ARM_ANGLED_DEPOSIT_POS = 0.5; // Claw tilted for front deposit

    private static double CLAW_MIN = 0;
    private static double CLAW_MAX = 0.2;
    public static double CLAW_GRAB_POS = 0.18;
    public static double CLAW_DROP_POS = 0.1;
    public static double CLAW_WIDE_POS = 0; // TODO: not wide enough

    private static double VSLIDE_MIN = 0.30;
    private static double VSLIDE_MAX = 0.61; // 0.62 will lock it at top
    public static double VSLIDE_LVL1_POS = 0.3;
    public static double VSLIDE_LVL2_POS = 0.36;
    public static double VSLIDE_LVL3_POS = 0.42;
    public static double VSLIDE_LVL4_POS = 0.47;
    public static double VSLIDE_LVL5_POS = 0.53;
    public static double VSLIDE_CLEAR_LVL1_POS = 0.49; // Lift cone above a 1 stack
    public static double VSLIDE_CLEAR_LVL2_POS = 0.53; // Lift cone above a 2 stack
    public static double VSLIDE_CLEAR_LVL3_POS = 0.61; // Lift cone above a 3 stack
    public static double VSLIDE_CLEAR_LVL4_POS = 0.61; // Lift cone above a 4 stack

    public Intake(HardwareMap hardwareMap) {
        this.extender = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "intakeExtender"), EXTENDER_PID);
        this.extender.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        this.arm = HardwareCreator.createServo(hardwareMap, "intakeArm", HardwareCreator.ServoType.AXON);
        this.claw = HardwareCreator.createServo(hardwareMap, "intakeClaw", HardwareCreator.ServoType.AXON);
        this.slide = HardwareCreator.createServo(hardwareMap, "intakeVSlide", HardwareCreator.ServoType.AXON);
    }

    public void initialize() {
        extendIn();
        armStore();
        clawClosed();
        slideDown();
    }

    public void update() {
        extender.update();
    }

    // Extender
    public void extendIn() {
        this.extender.setTargetPosition(EXTENDER_STORED_POS);
    }
    public void extendOut() {
        this.extender.setTargetPosition(EXTENDER_CYCLE_POS);
    }
    public void extendTransfer() {
        this.extender.setTargetPosition(EXTENDER_TRANSFER_POS);
    }

    // Arm
    public void armOut() {
        this.arm.setPosition(ARM_INTAKING_POS);
    }
    public void armStore() {
        this.arm.setPosition(ARM_STORED_POS);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER_POS);
    }

    // Claw
    public void clawOpen() {
        this.claw.setPosition(CLAW_WIDE_POS);
    }
    public void clawClosed() {
        this.claw.setPosition(CLAW_GRAB_POS);
    }

    // Slide
    public void slideUp() {
        this.slide.setPosition(VSLIDE_MAX);
    }
    public void slideDown() {
        this.slide.setPosition(VSLIDE_MIN);
    }
}
