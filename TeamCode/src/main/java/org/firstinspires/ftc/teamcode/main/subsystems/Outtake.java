package org.firstinspires.ftc.teamcode.main.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.AngleMotorWithPID;
import org.firstinspires.ftc.teamcode.util.hardware.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.DualMotorWithPID;
import org.firstinspires.ftc.teamcode.util.hardware.MagnetSwitchSensor;

@Config
public class Outtake {
    private final double TURRET_TICKS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28) * (68.0/10.0); // Goes through 435 RPM motor and then 68T/10T gear ratio
    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.006, 0, 0.0003); // was previously P=0.007
    private static double TURRET_MIN = -90; // TODO: Tune
    private static double TURRET_MAX = 90; // TODO: Tune
    public static int TURRET_CENTER = 0; // Initialize with turret centered
    public static int TURRET_LEFT = 52;
    public static int TURRET_RIGHT = -52;

    public static final double SLIDE_TICKS_PER_REV = ((1+(46.0/11.0)) * 28);
    public static final double SLIDE_ANGLE = 63;
    public static PIDCoefficients SLIDE_PID = new PIDCoefficients(0.010, 0, 0.0004);
    private static int SLIDE_MIN = 0;
    private static int SLIDE_MAX = 900;
    public static int SLIDE_HIGH = 600;
    public static int SLIDE_MID = 270;
    public static int SLIDE_LOW = 0;
    public static int SLIDE_STORED = 0;

    private static double ARM_MIN = 0;
    private static double ARM_MAX = 0.81;
    public static double ARM_FLAT_OUT = 0.06;
    public static double ARM_TILT_OUT = 0.09;
    public static double ARM_VERTICAL = 0.33; // might be unnecessary
    public static double ARM_TRANSFER = 0.72;
    public static double ARM_TRANSFER_AUTO_POS = 0.72;

    private static double LATCH_MIN = 0.28; // Guess (not that it matters)
    private static double LATCH_MAX = 0.85; // Guess
    public static double LATCH_OPEN = 0.42;
    public static double LATCH_BARELY = 0.88; // Just covers the bottom, does not apply pressure
    public static double LATCH_ENGAGED = 0.98; // may disable the servo temporarily

    private static double GUIDE_MIN = 0.17;
    private static double GUIDE_MAX = 0.86;
    public static double GUIDE_FLAT_OUT = 0.5;
    public static double GUIDE_RETRACT_DOWN = 0.83;
    public static double GUIDE_STORE_UP = 0.83; // 0.17 actually up
    public static double GUIDE_INIT = 0.83;

    private DualMotorWithPID slide;
    private AngleMotorWithPID turret;
    private Servo latch;
    private Servo arm;
    private Servo guide;
    private BeamBreakSensor guideSensor;
    private MagnetSwitchSensor slideSensor;

    private boolean outtakePidEnabled = true;

    public Outtake(HardwareMap hardwareMap) {
        this.turret = new AngleMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeTurret"), TURRET_TICKS_PER_REV, TURRET_PID);
        this.slide = new DualMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide1"), HardwareCreator.createMotor(hardwareMap, "outtakeSlide2"), true,
                SLIDE_PID, (x, y) -> (x > 50 ? 0.10 : 0.0)); // Feedforward, apply power of 0.10 when pos>50
        this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch", HardwareCreator.ServoType.DEFAULT);
        this.guide = HardwareCreator.createServo(hardwareMap, "outtakeGuide", HardwareCreator.ServoType.GOBILDA);
        this.arm = HardwareCreator.createServo(hardwareMap, "outtakeArm", HardwareCreator.ServoType.AXON);
        this.guideSensor = new BeamBreakSensor(hardwareMap, "guideBeamBreak");
        this.slideSensor = new MagnetSwitchSensor(hardwareMap, "outtakeMagnetSwitch");

        this.slide.setMaxPower(1.0);
    }

    public Outtake(HardwareMap hardwareMap, Outtake previousOuttake) {
        this.turret = previousOuttake.turret;
        this.slide = previousOuttake.slide;
        this.slide.setDirection(DcMotorSimple.Direction.FORWARD);
        this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch", HardwareCreator.ServoType.DEFAULT);
        this.guide = HardwareCreator.createServo(hardwareMap, "outtakeGuide", HardwareCreator.ServoType.GOBILDA);
        this.arm = HardwareCreator.createServo(hardwareMap, "outtakeArm", HardwareCreator.ServoType.AXON);
        this.guideSensor = new BeamBreakSensor(hardwareMap, "guideBeamBreak");
        this.slideSensor = new MagnetSwitchSensor(hardwareMap, "outtakeMagnetSwitch");

        this.turret.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.slide.getMainMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.slide.getSecondMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {
        store();
        guide.setPosition(GUIDE_INIT);
        turretCenter();
    }

    public void update() {
        turret.update();
        if (outtakePidEnabled) slide.update();
    }

    // Outtake Arm
    public void setArmPosition(double pos) {
        this.arm.setPosition(Range.clip(pos, ARM_MIN, ARM_MAX));
    }
    public void armFlatOut() {
        this.arm.setPosition(ARM_FLAT_OUT);
    }
    public void armTiltOut() {
        this.arm.setPosition(ARM_TILT_OUT);
    }
    public void armVertical() {
        this.arm.setPosition(ARM_VERTICAL);
    }
    public void armTransfer() {
        this.arm.setPosition(ARM_TRANSFER);
    }
    public void armTransferAuto() {
        this.arm.setPosition(ARM_TRANSFER_AUTO_POS);
    }
    public double getArmPosition() {
        return this.arm.getPosition();
    }

    // Latch movements
    public void setLatchPosition(double pos) {
        this.latch.setPosition(Range.clip(pos, LATCH_MIN, LATCH_MAX));
    }
    public void latchOpen() {
        this.latch.setPosition(LATCH_OPEN);
    }
    public void latchBarely() {
        this.latch.setPosition(LATCH_BARELY);
    }
    public void latchEngaged() {
        this.latch.setPosition(LATCH_ENGAGED);
    }
    public double getLatchPosition() {
        return this.latch.getPosition();
    }

    // Guide
    public void setGuidePosition(double pos) {
        this.guide.setPosition(Range.clip(pos, GUIDE_MIN, GUIDE_MAX));
    }
    public void guideFlatOut() {
        guide.setPosition(GUIDE_FLAT_OUT);
    }
    public void guideStoreUp() {
        guide.setPosition(GUIDE_STORE_UP);
    }
    public void guideRetractDown() {
        guide.setPosition(GUIDE_RETRACT_DOWN);
    }
    public double getGuidePosition() {
        return this.guide.getPosition();
    }

    // Slide movements
    public void setSlidePosition(int ticks) {
        this.slide.setTargetPosition(Range.clip(ticks, SLIDE_MIN, SLIDE_MAX));
    }
    public void slideStore() {
        Log.d("Outtake", "Slide lowered from " + getSlidePosition() + ", the prev target was " + getSlideTarget());
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
    public int getSlideTarget() {
        return this.slide.getTargetPosition();
    }
    public int getSlidePosition() {
        return this.slide.getCurrentPosition();
    }
    public double getSlidePower() {
        return this.slide.getPower();
    }
    public void setSlideMaxPower(double maxPower) {
        this.slide.setMaxPower(maxPower);
    }
    public DualMotorWithPID getSlide() {
        return this.slide;
    }
    public double slideTicksToInches(int ticks) {
        return ticks / SLIDE_TICKS_PER_REV;
    }

    // Turret movements
    public void setTurretAngle(double angle) {
        turret.setAngle(Range.clip(angle, TURRET_MIN, TURRET_MAX));
    }
    public void turretLeft() {
        turret.setAngle(TURRET_LEFT);
    }
    public void turretCenter() {
        Log.d("Outtake", "Turret centered from " + getTurretAngle() + ", the prev target was " + getTurretTarget());
        turret.setAngle(TURRET_CENTER);
    }
    public void turretRight() {
        turret.setAngle(TURRET_RIGHT);
    }
    public double getTurretTarget() {
        return this.turret.getTargetAngle();
    }
    public double getTurretAngle() {
        return this.turret.getAngle();
    }
    public double getTurretPower() {
        return this.turret.getPower();
    }
    public void setTurretMaxPower(double maxPower) {
        this.turret.setMaxPower(maxPower);
    }
    public AngleMotorWithPID getTurret() {
        return turret;
    }

    // Shortcuts
    public void store() {
        armTransfer();
        slideStore();
        guideStoreUp();
        latchOpen();
    }
    public void scheduleLatch() {
        new Thread(() -> { // TODO: Refactor this into planner
            try {
                Thread.sleep(20);
                latchBarely();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }
    public void raisePrep() {
        scheduleLatch();
        armTiltOut(); // TODO: Put in planner, delay until we reach near top of target
        guideFlatOut();
    }
    public void raiseMid() {
        raisePrep();
        slideMid();
    }
    public void raiseHigh() {
        raisePrep();
        slideHigh();
    }

    // *** overrides for outtake control *** //

    public void enableOuttakePID() {
        if (!outtakePidEnabled) {
            outtakePidEnabled = true;
            slide.setTargetPosition(slide.getCurrentPosition());
            slide.resetIntegralGain();
        }
    }

    public void disableOuttakePID() {
        if (outtakePidEnabled) {
            outtakePidEnabled = false;
        }
    }

    public void setOuttakeOverridePower(double power) {
        if (!outtakePidEnabled) {
            slide.getMainMotor().setPower(power);
            slide.getSecondMotor().setPower(power);
        }
    }

    public boolean isArmOut() {
        return arm.getPosition() < ARM_TILT_OUT + 0.1;
    }

    public boolean isSlideMagnetPresent() {
        return slideSensor.isMagnetPresent();
    }

    public boolean isSlideDown() {
        return slideSensor.isMagnetPresent() && Math.abs(slide.getVelocity()) < 3 && slide.getTargetPosition() < 5;
    }
}
