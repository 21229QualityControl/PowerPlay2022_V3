package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;
import java.util.function.BooleanSupplier;

public final class WaitSegment extends SequenceSegment {
    private BooleanSupplier test = () -> false;

    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super("WAIT", seconds, pose, pose, markers);
    }

    public WaitSegment(Pose2d pose, BooleanSupplier quitCase, double timeoutSeconds, List<TrajectoryMarker> markers) {
        super("WAIT-FOR", timeoutSeconds, pose, pose, markers);
        test = quitCase;
    }

    public final boolean test() {
        return test.getAsBoolean();
    }

    @Override
    public Pose2d getPose(double time) {
        return getStartPose();
    }
}
