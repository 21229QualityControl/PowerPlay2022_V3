package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import kotlin.jvm.functions.Function2;

/**
 * Class that represents an axon servo and its analog encoder.
 * Adds a PIDF controller.
 */
public class AxonServoWithPID {
    private CRServo crServo;
    private AnalogInput encoder;
    private PIDFController pidfController;
    private PIDCoefficients pid;
    private DcMotorSimple.Direction direction;
    private double min = 0;
    private double max = 1;
    private double tolerance = 0.03;
    private double maxPower = 0;
    private double lastSetPower = 0;

    public AxonServoWithPID(HardwareMap hardwareMap, String crServoName, String encoderName, PIDCoefficients pid) {
        this(hardwareMap, crServoName, encoderName, pid, (x, v) -> 0.0);
    }

    public AxonServoWithPID(HardwareMap hardwareMap, String crServoName, String encoderName,
                            PIDCoefficients pid, Function2<Double, Double, Double> f) {
        this.crServo = HardwareCreator.createCRServo(hardwareMap, crServoName, HardwareCreator.ServoType.AXON);
        this.encoder = hardwareMap.analogInput.get(encoderName);
        this.direction = DcMotorSimple.Direction.FORWARD;
        this.crServo.setDirection(this.direction);

        this.pid = pid;
        this.pidfController = new PIDFController(pid, 0, 0, 0, f);
        this.pidfController.setOutputBounds(-1, 1);
        this.pidfController.setTargetPosition(0.5);
    }

    public void update() {
        lastSetPower = this.pidfController.update(getRawEncoderPosition());
        crServo.setPower(lastSetPower);
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
        this.pidfController.setOutputBounds(-this.maxPower, this.maxPower);
    }

    public void setPosition(double pos) {
        this.pidfController.setTargetPosition(Range.scale(pos, min, max, 0, 1));
    }

    public void scaleRange(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    public double getRawEncoderPosition() {
        if (direction == DcMotorSimple.Direction.REVERSE) {
            return 1 - encoder.getVoltage() / 3.3;
        }
        return encoder.getVoltage() / 3.3;
    }

    public double getTargetPosition() {
        return pidfController.getTargetPosition();
    }

    public double getLastSetPower() {
        return lastSetPower;
    }

    public void setTargetPositionTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getTargetPositionTolerance() {
        return tolerance;
    }

    public boolean isBusy() {
        return Math.abs(pidfController.getLastError()) > tolerance || Math.abs(pidfController.getTargetVelocity()) > tolerance;
    }

    public void resetIntegralGain() {
        this.pidfController.reset();
    }
}
