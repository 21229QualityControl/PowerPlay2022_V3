package org.firstinspires.ftc.teamcode.main.subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.roadrunner.PositionMaintainer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StateCopyLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.data.CPose2d;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;

import java.util.Arrays;
import java.util.List;

/**
 * Wrapper class for roadrunner.
 *
 * Modified from Roadrunner Quickstart's SampleMecanumDrive
 */
@Config
public class Roadrunner extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 2); // D here acts as a P for pose velocity
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static PIDCoefficients KEEP_POSITION_TRANSLATIONAL_PID = new PIDCoefficients(10, 0.5, 0);
    public static PIDCoefficients KEEP_POSITION_HEADING_PID = new PIDCoefficients(11, 0.5, 0);
    public static CPose2d KEEP_POSITION_TOLERANCE = new CPose2d(0.5, 0.5, Math.toRadians(2.0));

    public static double LATERAL_MULTIPLIER = 1.72; // reality/expected for strafing effort

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private TrajectoryFollower follower;
    private PositionMaintainer positionMaintainer;
    private Hub hub;
    private Drivetrain drivetrain;

    public Roadrunner(HardwareMap hardwareMap, Hub hub, Drivetrain drivetrain) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.05);

        positionMaintainer = new PositionMaintainer(KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_TRANSLATIONAL_PID, KEEP_POSITION_HEADING_PID, KEEP_POSITION_TOLERANCE.asPose2d());

        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, hub));

        // copy position from roadrunner if motors are disabled
        if (HardwareCreator.SIMULATE_DRIVETRAIN || HardwareCreator.SIMULATE_HARDWARE) setLocalizer(new StateCopyLocalizer());

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, positionMaintainer, HEADING_PID);

        this.hub = hub;
        this.drivetrain = drivetrain;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turnToAsync(double angle) {
        turnAsync(Angle.normDelta(angle - getPoseEstimate().getHeading()));
    }

    public void turnTo(double angle) {
        turn(Angle.normDelta(angle - getPoseEstimate().getHeading()));
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null){
            StateCopyLocalizer.vel = signal.getVel();
            StateCopyLocalizer.accel = signal.getAccel();
            this.setDriveSignal(signal);
        }
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public void stopTrajectory() {
        trajectorySequenceRunner.stop(); // runs all remaining markers
        drivetrain.setMotorPowers(0, 0, 0, 0);
        Log.d("Roadrunner", "User requested stop");
    }
    public void forceStopTrajectory() {
        trajectorySequenceRunner.forceStop(); // ignores all remaining markers
        drivetrain.setMotorPowers(0, 0, 0, 0);
        Log.d("Roadrunner", "User requested force stop");
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public boolean isPositionMaintainerBusy() {
        return trajectorySequenceRunner.isPositionMaintainerBusy();
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        drivetrain.setPIDFCoefficients(runMode, coefficients);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return drivetrain.getWheelPositions();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return drivetrain.getWheelVelocities();
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drivetrain.setMotorPowers(v, v1, v2, v3);
    }

    @Override
    public double getRawExternalHeading() {
        return hub.getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return hub.getExternalHeadingVelocity();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel) {
        return getVelocityConstraint(maxVel, MAX_ANG_VEL, TRACK_WIDTH);
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return trajectorySequenceRunner;
    }
}
