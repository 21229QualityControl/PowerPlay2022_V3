package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

/**
 * Superclass for run-in-place segments to determine whether to actively maintain position.
 */
public abstract class StationarySegment extends SequenceSegment{
    private boolean keepPosition;

    protected StationarySegment(String name, double duration, Pose2d pose, boolean keepPosition, List<TrajectoryMarker> markers) {
        super(name, duration, pose, pose, markers);
        this.keepPosition = keepPosition;
    }

    public boolean isKeepingPosition() {
        return keepPosition;
    }

    @Override
    public Pose2d getPose(double time) {
        return getStartPose();
    }
}
