package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithRunToPositionPID;

@Config
public class Intake {
    private MotorWithRunToPositionPID extender;
    private Servo arm;
    private Servo claw;
    private Servo vslide;

    public static final double EXTENDER_TICKS_PER_REV = ((1+(46.0/11.0)) * 28);
    public static final double EXTENDER_SPOOL_CIRCUMFERENCE = 112 / 25.4; // 112mm
    public static PIDFCoefficients EXTENDER_VEL_PIDF = new PIDFCoefficients(23, 0.005, 5, 0.0);
    public static double EXTENDER_POS_P = 23;
    private static int EXTENDER_MIN = 0;
    private static int EXTENDER_MAX = 1050; // 1080 is technically the max
    public static int EXTENDER_STORED_POS = 0;
    public static int EXTENDER_CYCLE_TELEOP_POS = 530;
    public static int EXTENDER_BEFORE_CYCLE_TELEOP_POS = 300;
    public static int EXTENDER_CYCLE_POS = 560;
    public static int EXTENDER_BEFORE_STACK_POS = 500;
    public static int EXTENDER_TRANSFER_AUTO_POS = 55;

    private static double ARM_MIN = 0.137;
    private static double ARM_MAX = 0.585;
    public static double ARM_PARKING_POS = 0.22; // Arm inside robot, slightly angled in
    public static double ARM_STORED_POS = 0.240; // Arm as vertical as possible (without blocking outtake)
    public static double ARM_INTAKING_POS = 0.56; // Claw leveled
    public static double ARM_TRANSFER_POS = 0.22;
    public static double ARM_TRANSFER_AUTO_POS = 0.20;
    public static double ARM_ANGLED_DEPOSIT_POS = 0.42; // Claw tilted for front deposit on low junction
    public static double ARM_TWO_CONE = 0.32;

    private static double CLAW_MIN = 0.55;
    private static double CLAW_MAX = 1;
    public static double CLAW_CLOSED_POS = 0.55; // closed completely
    public static double CLAW_GRAB_POS = 0.55;
    public static double CLAW_RELEASE_POS = 0.88; // open quite wide but can still fit through intake
    public static double CLAW_WIDE_POS = 1;

    private static double VSLIDE_MIN = 0.27;
    private static double VSLIDE_MAX = 0.61; // 0.62 will lock it at top
    public static double VSLIDE_TRANSFER_POS = 0.38;
    public static double VSLIDE_TRANSFER_AUTO_POS = 0.44;
    public static double VSLIDE_LVL1_POS = 0.27;
    public static double VSLIDE_LVL2_POS = 0.34;
    public static double VSLIDE_LVL3_POS = 0.39;
    public static double VSLIDE_LVL4_POS = 0.45;
    public static double VSLIDE_LVL5_POS = 0.51;
    public static double VSLIDE_CLEAR_LVL0_POS = 0.61; // Lift cone off the ground
    public static double VSLIDE_CLEAR_LVL1_POS = 0.61; // Lift cone above a 1 stack
    public static double VSLIDE_CLEAR_LVL2_POS = 0.61; // Lift cone above a 2 stack
    public static double VSLIDE_CLEAR_LVL3_POS = 0.61; // Lift cone above a 3 stack
    public static double VSLIDE_CLEAR_LVL4_POS = 0.61; // Lift cone above a 4 stack

    public Intake(HardwareMap hardwareMap) {
        this.extender = new MotorWithRunToPositionPID(HardwareCreator.createMotor(hardwareMap, "intakeExtender"), EXTENDER_VEL_PIDF, EXTENDER_POS_P);
        this.extender.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        this.arm = HardwareCreator.createServo(hardwareMap, "intakeArm", HardwareCreator.ServoType.AXON);
        this.claw = HardwareCreator.createServo(hardwareMap, "intakeClaw", HardwareCreator.ServoType.AXON);
        this.vslide = HardwareCreator.createServo(hardwareMap, "intakeVSlide", HardwareCreator.ServoType.AXON);

        this.extender.setMaxPower(0.9);
        this.extender.getMotor().setCurrentAlert(8000, CurrentUnit.MILLIAMPS);
    }

    public void initialize() {
        extendStore();
        armStore();
        clawRelease();
        vslideDown();
    }

    public void update() {
        Dashboard.packet.put("extenderPosition", getExtenderPosition());
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
    public void extendTransferAuto() {
        this.extender.setTargetPosition(EXTENDER_TRANSFER_AUTO_POS);
    }
    public int getExtenderTarget() {
        return this.extender.getTargetPosition();
    }
    public int getExtenderPosition() {
        return this.extender.getCurrentPosition();
    }
    public void setExtenderMaxPower(double maxPower) {
        this.extender.setMaxPower(maxPower);
    }
    public MotorWithRunToPositionPID getExtender() {
        return this.extender;
    }
    public double extenderTicksToInches(int ticks) {
        return ticks / EXTENDER_TICKS_PER_REV * EXTENDER_SPOOL_CIRCUMFERENCE;
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
    public void armTransferAuto() {
        this.arm.setPosition(ARM_TRANSFER_AUTO_POS);
    }
    public void armAngledDeposit() {
        this.arm.setPosition(ARM_ANGLED_DEPOSIT_POS);
    }
    public void armPark() {
        this.arm.setPosition(ARM_PARKING_POS);
    }
    public void armTwoCone() {
        this.arm.setPosition(ARM_TWO_CONE);
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
    public void vslideTransferAuto() {
        this.vslide.setPosition(VSLIDE_TRANSFER_AUTO_POS);
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
     * @return is the extender completely in the robot
     */
    public boolean isExtenderIn() {
        return extender.getCurrentPosition() < EXTENDER_STORED_POS + 10;
    }

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
        return Math.abs(claw.getPosition() - CLAW_WIDE_POS) < 0.01;
    }

    /**
     * @return is the claw closed for grabbing
     */
    public boolean isClawClosed() {
        return Math.abs(claw.getPosition() - CLAW_GRAB_POS) < 0.01;
    }

    /**
     * @return is the vslide up in any unstacking position
     */
    public boolean isVSlideUp() {
        return vslide.getPosition() > VSLIDE_MIN + 0.01;
    }
}
