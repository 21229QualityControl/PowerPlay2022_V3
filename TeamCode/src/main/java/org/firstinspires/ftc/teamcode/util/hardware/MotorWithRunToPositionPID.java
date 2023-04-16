package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import kotlin.jvm.functions.Function2;

public class MotorWithRunToPositionPID {
    private DcMotorEx motor;
    private PIDFCoefficients velocityPIDF;
    private double positionP;
    private int targetPosition = 0;
    private int internalOffset = 0;
    private int tolerance = 5;
    private double maxPower = 0;

    public MotorWithRunToPositionPID(DcMotorEx motor, PIDFCoefficients velocityPIDF, double positionP) {
        this.motor = motor;
        this.velocityPIDF = velocityPIDF;
        this.positionP = positionP;

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        motor.setPositionPIDFCoefficients(positionP);

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0);
    }

    public DcMotorEx getMotor() {
        return motor;
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
        this.motor.setTargetPosition(position - internalOffset); // TODO: Verify sign
    }

    /**
     * Returns the current reading of the encoder for this motor in encoder ticks.
     * @return the current reading of the encoder for this motor
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() + internalOffset;
    }

    public void zeroMotorInternals() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    public boolean isBusy() {
        return motor.isBusy();
    }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     */
    public int getTargetPosition() {
        return motor.getTargetPosition();
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
        motor.setPower(maxPower);
    }

    /**
     * Returns the maximum power level that can be sent to the motor
     * @return the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public double getMaxPower() {
        return maxPower;
    }

    /**
     * Sets the motor power to zero
     */
    public void stopMotor() {
        motor.setPower(0);
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
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
