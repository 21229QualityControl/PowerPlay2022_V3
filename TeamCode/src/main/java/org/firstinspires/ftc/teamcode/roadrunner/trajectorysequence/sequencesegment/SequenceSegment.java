package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public abstract class SequenceSegment {
    private final String name;
    private final double duration; // seconds
    private final Pose2d startPose;
    private final Pose2d endPose;
    protected List<TrajectoryMarker> markers;

    protected SequenceSegment(
            String name,
            double duration,
            Pose2d startPose, Pose2d endPose,
            List<TrajectoryMarker> markers
    ) {
        this.name = name;
        this.duration = duration;
        this.startPose = startPose;
        this.endPose = endPose;
        this.markers = markers;
    }

    public String getName() {
        return name;
    }

    public double getDuration() {
        return this.duration;
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    public List<TrajectoryMarker> getMarkers() {
        return markers;
    }

    public abstract Pose2d getPose(double time);
}
