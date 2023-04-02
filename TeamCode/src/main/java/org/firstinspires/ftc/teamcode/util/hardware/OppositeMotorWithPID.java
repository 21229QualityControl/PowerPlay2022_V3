package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import kotlin.jvm.functions.Function2;

public class OppositeMotorWithPID {
    private DcMotorEx motor;
    private DcMotorEx oppositeMotor;
    private PIDFController pidfController;
    private PIDCoefficients pid;
    private int targetPosition = 0;
    private int internalOffset = 0;
    private int tolerance = 10;
    private double maxPower = 1e-8; // zero does not work

    public OppositeMotorWithPID(DcMotorEx motor, DcMotorEx oppositeMotor, PIDCoefficients pid) {
        this(motor, oppositeMotor, pid, (x, v) -> 0.0);
    }

    public OppositeMotorWithPID(DcMotorEx motor, DcMotorEx oppositeMotor, PIDCoefficients pid, Function2<Double, Double, Double> f) {
        this.motor = motor;
        this.oppositeMotor = oppositeMotor;
        this.pid = pid;
        this.pidfController = new PIDFController(pid, 0, 0, 0, f);
        this.pidfController.setOutputBounds(-maxPower, maxPower);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppositeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public DcMotorEx getOppositeMotor() {
        return oppositeMotor;
    }

    /**
     * Updates the power sent to the motor according to the pidf controller.
     */
    public void update() {
        double newPower = this.pidfController.update(motor.getCurrentPosition(), motor.getVelocity());
        motor.setPower(newPower);
        oppositeMotor.setPower(-newPower);
    }

    /**
     * Update the PID values in the controller.
     * Note that it is not possible to replace f after instantiation
     * @param newPID the new pid values to use
     */
    public void setPIDCoefficients(PIDCoefficients newPID) {
        this.pid.kP = newPID.kP;
        this.pid.kI = newPID.kI;
        this.pid.kD = newPID.kD;
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold there at. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * @param position the desired encoder target position
     */
    public void setTargetPosition(int position) {
        this.targetPosition = position;
        this.pidfController.setTargetPosition(position + internalOffset);
    }

    /**
     * Returns the current reading of the encoder for this motor in encoder ticks.
     * @return the current reading of the encoder for this motor
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() + internalOffset;
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    public boolean isBusy() {
        return Math.abs(pidfController.getLastError()) > tolerance || Math.abs(pidfController.getTargetVelocity()) > tolerance;
    }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Sets the maximum power level that can be sent to the motor
     * @param maxPower the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
        this.pidfController.setOutputBounds(-this.maxPower, this.maxPower);
    }

    /**
     * Returns the maximum power level that can be sent to the motor
     * @return the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public double getMaxPower() {
        return maxPower;
    }

    /**
     * Returns the current power level sent to the motor.
     * @return the current level of the motor, a value in the interval [-1.0, 1.0]
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        oppositeMotor.setDirection(direction.inverted());
    }

    /**
     * Remaps the current position to the given position
     * @param currentPosition the position to remap as
     */
    public void setCurrentPosition(int currentPosition) {
        this.internalOffset = currentPosition - motor.getCurrentPosition(); // raw + offset = current
    }

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    public void setTargetPositionTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    public int getTargetPositionTolerance() {
        return tolerance;
    }
}
