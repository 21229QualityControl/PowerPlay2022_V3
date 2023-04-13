/*
 * Based off of Acmerobotic's com.acmerobotics.roadrunner.followers.TrajectoryFollower.kt
 */
package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.util.Angle;

public class PositionMaintainer {
    private final PIDFController axialController;
    private final PIDFController lateralController;
    private final PIDFController headingController;
    private Pose2d admissibleError;
    private final Pose2d zero = new Pose2d();
    private Pose2d targetPose = new Pose2d();
    private Pose2d lastError = new Pose2d();

    public PositionMaintainer(PIDCoefficients axialCoeffs, PIDCoefficients lateralCoeffs, PIDCoefficients headingCoeffs, Pose2d admissibleError) {
        this.axialController = new PIDFController(axialCoeffs);
        this.lateralController = new PIDFController(lateralCoeffs);
        this.headingController = new PIDFController(headingCoeffs);
        this.admissibleError = admissibleError;

        this.headingController.setInputBounds(-Math.PI, Math.PI);
    }

    public void maintainPosition(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public void resetController() {
        axialController.reset();
        lateralController.reset();
        headingController.reset();
    }

    public boolean isBusy() {
        boolean busy = Math.abs(lastError.getX()) > admissibleError.getX() ||
                Math.abs(lastError.getY()) > admissibleError.getY() ||
                Math.abs(Angle.normDelta(lastError.getHeading())) > admissibleError.getHeading();
//        Log.d("PositionMaintainer", "Busy: " + busy + ", Last error: " + lastError.toString());
        return busy;
    }

    public DriveSignal update(Pose2d currentPose, Pose2d currentRobotVel) {
        Pose2d poseError = Kinematics.calculateRobotPoseError(targetPose, currentPose);

        boolean admissible = Math.abs(poseError.getX()) <= admissibleError.getX() &&
                Math.abs(poseError.getY()) <= admissibleError.getY() &&
                Math.abs(Angle.normDelta(poseError.getHeading())) <= admissibleError.getHeading();

        if (admissible) {
            resetController(); // reset so that integration won't jerk the controller
            return new DriveSignal();
        } else {
            // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
            axialController.setTargetPosition(poseError.getX());
            lateralController.setTargetPosition(poseError.getY());
            headingController.setTargetPosition(poseError.getHeading());

            // note: feedforward is processed at the wheel level
            double axialCorrection = axialController.update(0.0, currentRobotVel.getX());
            double lateralCorrection = lateralController.update(0.0, currentRobotVel.getY());
            double headingCorrection = headingController.update(0.0, currentRobotVel.getHeading());

            Pose2d correctedVelocity = new Pose2d(
                    axialCorrection,
                    lateralCorrection,
                    headingCorrection
            );

            lastError = poseError;

//            Log.d("PositionMaintainer", "Corrected velocity: " + correctedVelocity.toString());
            return new DriveSignal(correctedVelocity, zero);
        }
    }

    public Pose2d getLastError() {
        return lastError;
    }

    public Pose2d getMaintainedPosition() {
        return targetPose;
    }
}