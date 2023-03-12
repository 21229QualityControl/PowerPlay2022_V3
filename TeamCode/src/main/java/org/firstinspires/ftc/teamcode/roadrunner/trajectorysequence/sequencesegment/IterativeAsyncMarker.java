package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.ActiveIterativeAsyncManager;

import java.util.List;

/**
 * Marker that starts and ends an iterative function.
 * Functions are managed by ActiveIterativeAsyncManager
 */
public class IterativeAsyncMarker extends SequenceSegment {

    private String name;
    private Runnable function;

    public IterativeAsyncMarker(String name, Runnable asyncFunction, Pose2d startPose, List<TrajectoryMarker> markers) {
        super("ITERATIVE_ASYNC_START", 0, startPose, startPose, markers);
        this.name = name;
        this.function = asyncFunction;
    }

    public IterativeAsyncMarker(String name, Pose2d startPose, List<TrajectoryMarker> markers) {
        super("ITERATIVE_ASYNC_END", 0, startPose, startPose, markers);
        this.name = name;
        this.function = null;
    }

    public void process() {
        if (function != null) {
            start();
        } else {
            end();
        }
    }

    public boolean start() {
        return ActiveIterativeAsyncManager.add(name, function);
    }

    public boolean end() {
        return ActiveIterativeAsyncManager.remove(name);
    }

    @Override
    public Pose2d getPose(double time) {
        return getStartPose();
    }
}
