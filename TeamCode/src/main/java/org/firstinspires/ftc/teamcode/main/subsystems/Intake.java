package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;

@Config
public class Intake {
    private MotorWithPID extender;
    private Servo arm;
    private Servo claw;
    private Servo vslide;

    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(0.010, 0, 0.0001);
    private static int EXTENDER_MIN = 0;
    private static int EXTENDER_MAX = 1050; // 1080 is technically the max
    public static int EXTENDER_STORED_POS = 0;
    public static int EXTENDER_CYCLE_POS = 560; // 560 for auto
    public static int EXTENDER_CYCLE_TELEOP_POS = 510;
    public static int EXTENDER_BEFORE_STACK_POS = 460;

    private static double ARM_MIN = 0;
    private static double ARM_MAX = 0.87;
    public static double ARM_STORED_POS = 0.30; // Arm is placed vertical
    public static double ARM_INTAKING_POS = 0.595; // Claw leveled
    public static double ARM_TRANSFER_POS = 0.25;
    public static double ARM_TRANSFER_COMPLETE_POS = 0.33;
    public static double ARM_ANGLED_DEPOSIT_POS = 0.5; // Claw tilted for front deposit

    private static double CLAW_MIN = 0.14;
    private static double CLAW_MAX = 0.67;
    public static double CLAW_CLOSED_POS = 0.14; // closed completely
    public static double CLAW_GRAB_POS = 0.14;
    public static double CLAW_RELEASE_POS = 0.48; // open quite wide but can still fit through intake
    public static double CLAW_WIDE_POS = 0.67;

    private static double VSLIDE_MIN = 0.30;
    private static double VSLIDE_MAX = 0.61; // 0.62 will lock it at top
    public static double VSLIDE_TRANSFER_POS = 0.4;
    public static double VSLIDE_LVL1_POS = 0.3;
    public static double VSLIDE_LVL2_POS = 0.36;
    public static double VSLIDE_LVL3_POS = 0.42;
    public static double VSLIDE_LVL4_POS = 0.47;
    public static double VSLIDE_LVL5_POS = 0.53;
    public static double VSLIDE_CLEAR_LVL0_POS = 0.49; // Lift cone off the ground
    public static double VSLIDE_CLEAR_LVL1_POS = 0.49; // Lift cone above a 1 stack
    public static double VSLIDE_CLEAR_LVL2_POS = 0.53; // Lift cone above a 2 stack
    public static double VSLIDE_CLEAR_LVL3_POS = 0.61; // Lift cone above a 3 stack
    public static double VSLIDE_CLEAR_LVL4_POS = 0.61; // Lift cone above a 4 stack

    public Intake(HardwareMap hardwareMap) {
        this.extender = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "intakeExtender"), EXTENDER_PID);
        this.extender.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        this.arm = HardwareCreator.createServo(hardwareMap, "intakeArm", HardwareCreator.ServoType.AXON);
        this.claw = HardwareCreator.createServo(hardwareMap, "intakeClaw", HardwareCreator.ServoType.AXON);
        this.vslide = HardwareCreator.createServo(hardwareMap, "intakeVSlide", HardwareCreator.ServoType.AXON);

        this.extender.setMaxPower(1.0);
    }

    public void initialize() {
        extendStore();
        armStore();
        clawRelease();
        vslideDown();
    }

    public void update() {
        extender.update();
    }

    // Extender
    public void extenderTo(int ticks) {
        this.extender.setTargetPosition(Range.clip(ticks, EXTENDER_MIN, EXTENDER_MAX));
    }
    public void extendStore() {
        this.extender.setTargetPosition(EXTENDER_STORED_POS);
    }
    public void extendCycle() {
        this.extender.setTargetPosition(EXTENDER_CYCLE_POS);
    }
    public void extendCycleTeleop() {
        this.extender.setTargetPosition(EXTENDER_CYCLE_TELEOP_POS);
    }
    public int getExtenderTarget() {
        return this.extender.getTargetPosition();
    }
    public int getExtenderPosition() {
        return this.extender.getCurrentPosition();
    }
    public double getExtenderPower() {
        return this.extender.getPower();
    }
    public void setExtenderMaxPower(double maxPower) {
        this.extender.setMaxPower(maxPower);
    }
    public MotorWithPID getExtender() {
        return this.extender;
    }

    // Intake Arm
    public void setArmPosition(double pos) {
        this.arm.setPosition(Range.clip(pos, ARM_MIN, ARM_MAX));
    }
    public void armIntake() {
        this.arm.setPosition(ARM_INTAKING_POS);
    }
    public void armStore() {
        this.arm.setPosition(ARM_STORED_POS);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER_POS);
    }
    public void armTransferComplete() {
        this.arm.setPosition(ARM_TRANSFER_COMPLETE_POS);
    }
    public void armAngledDeposit() {
        this.arm.setPosition(ARM_ANGLED_DEPOSIT_POS);
    }
    public double getArmPosition() {
        return this.arm.getPosition();
    }

    // Claw
    public void setClawPosition(double pos) {
        this.claw.setPosition(Range.clip(pos, CLAW_MIN, CLAW_MAX));
    }
    public void clawWide() {
        this.claw.setPosition(CLAW_WIDE_POS);
    }
    public void clawRelease() {
        this.claw.setPosition(CLAW_RELEASE_POS);
    }
    public void clawGrab() {
        this.claw.setPosition(CLAW_GRAB_POS);
    }
    public void clawClosed() {
        this.claw.setPosition(CLAW_CLOSED_POS);
    }
    public double getClawPosition() {
        return this.claw.getPosition();
    }

    // Slide
    public void setVSlidePosition(double pos) {
        this.vslide.setPosition(Range.clip(pos, VSLIDE_MIN, VSLIDE_MAX));
    }
    public void vslideDown() {
        this.vslide.setPosition(VSLIDE_MIN);
    }
    public void vslideTop() {
        this.vslide.setPosition(VSLIDE_MAX);
    }
    public void vslideTransfer() {
        this.vslide.setPosition(VSLIDE_TRANSFER_POS);
    }
    public void vslideLevel(int level) {
        switch (level) {
            case 1:
                this.vslide.setPosition(VSLIDE_LVL1_POS);
                break;
            case 2:
                this.vslide.setPosition(VSLIDE_LVL2_POS);
                break;
            case 3:
                this.vslide.setPosition(VSLIDE_LVL3_POS);
                break;
            case 4:
                this.vslide.setPosition(VSLIDE_LVL4_POS);
                break;
            case 5:
                this.vslide.setPosition(VSLIDE_LVL5_POS);
                break;
            default:
                throw new RuntimeException("Unsupported vslide level: " + level);
        }
    }
    public void vslideLiftLevel(int level) {
        switch (level) {
            case 1:
                this.vslide.setPosition(VSLIDE_CLEAR_LVL0_POS);
                break;
            case 2:
                this.vslide.setPosition(VSLIDE_CLEAR_LVL1_POS);
                break;
            case 3:
                this.vslide.setPosition(VSLIDE_CLEAR_LVL2_POS);
                break;
            case 4:
                this.vslide.setPosition(VSLIDE_CLEAR_LVL3_POS);
                break;
            case 5:
                this.vslide.setPosition(VSLIDE_CLEAR_LVL4_POS);
                break;
            default:
                throw new RuntimeException("Unsupported vslide lift level: " + level);
        }
    }
    public double getVSlidePosition() {
        return this.vslide.getPosition();
    }



    // Some state checks
    /**
     * @return is the arm down in an intaking position
     */
    public boolean isArmFlat() {
        return arm.getPosition() > ARM_INTAKING_POS - 0.05;
    }

    /**
     * @return is the arm outside the robot, i.e past vertical
     */
    public boolean isArmOut() {
        return arm.getPosition() > ARM_STORED_POS + 0.01;
    }

    /**
     * @return is the arm blocking the outtake, i.e over it
     */
    public boolean isArmBlockingOuttake() {
        return arm.getPosition() < ARM_STORED_POS - 0.01;
    }

    /**
     * @return is the claw open wide for grabbing
     */
    public boolean isClawOpen() {
        return claw.getPosition() > CLAW_WIDE_POS - 0.01;
    }

    /**
     * @return is the claw closed for grabbing
     */
    public boolean isClawClosed() {
        return claw.getPosition() < CLAW_GRAB_POS + 0.01;
    }

    /**
     * @return is the vslide up in any unstacking position
     */
    public boolean isVSlideUp() {
        return vslide.getPosition() > VSLIDE_MIN + 0.01;
    }
}
