package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;
import java.util.function.BooleanSupplier;

public final class WaitSegment extends StationarySegment {
    private BooleanSupplier test = () -> false;

    public WaitSegment(Pose2d pose, double seconds, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("WAIT", seconds, pose, keepPosition, markers);
    }

    public WaitSegment(Pose2d pose, BooleanSupplier quitCase, double timeoutSeconds, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("WAIT-FOR", timeoutSeconds, pose, keepPosition, markers);
        test = quitCase;
    }

    public final boolean test() {
        return test.getAsBoolean();
    }
}
