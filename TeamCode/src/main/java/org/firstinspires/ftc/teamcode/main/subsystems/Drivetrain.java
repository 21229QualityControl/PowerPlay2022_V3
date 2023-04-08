package org.firstinspires.ftc.teamcode.main.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.fakes.DcMotorFake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Wrapper class for our drivetrain.
 *
 * Modified from Roadrunner Quickstart's SampleMecanumDrive
 */
public class Drivetrain {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private Hub hub; // needed to get the voltage

    public Drivetrain(HardwareMap hardwareMap, Hub hub) {
        this.hub = hub;

        if (HardwareCreator.SIMULATE_DRIVETRAIN) {
            leftFront = new DcMotorFake();
            leftRear = new DcMotorFake();
            rightRear = new DcMotorFake();
            rightFront = new DcMotorFake();
        } else {
            leftFront = HardwareCreator.createMotor(hardwareMap, "leftFront");
            leftRear = HardwareCreator.createMotor(hardwareMap, "leftRear");
            rightRear = HardwareCreator.createMotor(hardwareMap, "rightRear");
            rightFront = HardwareCreator.createMotor(hardwareMap, "rightFront");
        }

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setMaxRPM(DriveConstants.MAX_RPM);
            motorConfigurationType.setTicksPerRev(DriveConstants.TICKS_PER_REV / DriveConstants.GEAR_RATIO);
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / hub.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    @NonNull
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public List<Double> getMotorPowers() {
        List<Double> wheelPowers = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPowers.add(motor.getPower());
        }
        return wheelPowers;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
}