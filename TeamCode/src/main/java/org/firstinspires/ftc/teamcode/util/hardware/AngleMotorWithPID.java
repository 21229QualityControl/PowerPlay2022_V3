package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/**
 * Angle Motor that uses our PID controller
 */
public class AngleMotorWithPID extends AngleMotor {
   private PIDFController pidfController;
   private PIDCoefficients pid;
   private double maxPower = 1;

   public AngleMotorWithPID(DcMotorEx motor, double ticksPerRev, PIDCoefficients pid) {
      super(motor, ticksPerRev);
      this.pid = pid;

      this.pidfController = new PIDFController(pid);
      this.pidfController.setOutputBounds(-1, 1);

      motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

   /**
    * If you stop updating, don't forget to call stopMotor()
    */
   public void update() {
      motor.setPower(this.pidfController.update(getCurrentPosition(), motor.getVelocity()));
   }

   @Override
   public void setPower(double power) {
      setMaxPower(maxPower);
   }

   public void setMaxPower(double maxPower) {
      this.maxPower = Math.abs(maxPower);
      this.pidfController.setOutputBounds(-this.maxPower, this.maxPower);
   }

   public double getMaxPower() {
      return maxPower;
   }

   public double getPower() {
      return motor.getPower();
   }

   public void stopMotor() {
      motor.setPower(0);
   }

   @Override
   public void setAngle(double targetAngle) {
      this.targetAngle = targetAngle;
      this.pidfController.setTargetPosition(angleToTicks(targetAngle));
   }

   public void resetIntegralGain() {
      this.pidfController.reset();
   }
}