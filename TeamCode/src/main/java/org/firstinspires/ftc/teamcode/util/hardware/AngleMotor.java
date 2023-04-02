package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Abstract class for a motor that uses degrees instead of encoder ticks
 * Needs setAngle() implemented
 */
public abstract class AngleMotor {
   protected DcMotorEx motor;
   protected double ticksPerRev;

   protected int tickOffset = 0; // not for return use
   protected double targetAngle = 0;

   public AngleMotor(DcMotorEx motor, double ticksPerRev) {
      this.motor = motor;
      this.ticksPerRev = ticksPerRev;
   }

   public DcMotorEx getMotor() {
      return motor;
   }

   public void setInternalAngle(double currentAngle) {
      tickOffset = angleToTicks(currentAngle) - motor.getCurrentPosition();
   }

   public double getAngle() {
      return ticksToAngle(motor.getCurrentPosition() + tickOffset);
   }

   public double getAngularVelocity() {
      return ticksToAngle(motor.getVelocity()); // degrees per second
   }

   public double getTargetAngle() {
      return targetAngle;
   }

   public int getCurrentPosition() {
      return motor.getCurrentPosition() + tickOffset;
   }

   public int getTargetPosition() {
      return angleToTicks(targetAngle);
   }

   public abstract void setAngle(double targetAngle);

   public void setPower(double power) {
      motor.setPower(power);
   }

   public double getPower() {
      return motor.getPower();
   }

   public void setDirection(DcMotorSimple.Direction direction) {
      motor.setDirection(direction);
   }

   protected double ticksToAngle(double ticks) {
      return ticks / ticksPerRev * 360;
   }

   protected int angleToTicks(double degrees) {
      return (int) (degrees / 360 * ticksPerRev);
   }
}