package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.ActiveIterativeAsyncManager;

import java.util.List;

/**
 * Marker that starts and ends an iterative function.
 * Functions are managed by ActiveIterativeAsyncManager
 */
public class IterativeAsyncMarker extends StationarySegment {

    private String name;
    private Runnable function;
    private boolean isRemove;

    public IterativeAsyncMarker(String name, Runnable asyncFunction, Pose2d pose, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("ITERATIVE_ASYNC_START", 0, pose, keepPosition, markers);
        this.name = name;
        this.function = asyncFunction;
        this.isRemove = false;
    }

    public IterativeAsyncMarker(String name, Pose2d pose, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("ITERATIVE_ASYNC_END", 0, pose, keepPosition, markers);
        this.name = name;
        this.function = null;
        this.isRemove = true;
    }

    public boolean isRemove() {
        return isRemove;
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
