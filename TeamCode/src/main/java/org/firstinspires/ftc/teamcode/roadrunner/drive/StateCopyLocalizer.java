package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

/**
 * Localizer for run-in-place simulations
 * Also useful for remembering where the last pose from the follower was
 */
public class StateCopyLocalizer implements Localizer {
    public static Pose2d pose = new Pose2d();
    public static Pose2d vel = new Pose2d();
    public static Pose2d accel = new Pose2d();


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return pose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        pose = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return vel;
    }

    @Override
    public void update() {}
}
