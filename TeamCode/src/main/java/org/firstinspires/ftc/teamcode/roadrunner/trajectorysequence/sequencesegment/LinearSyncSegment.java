package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Segment that runs code. While being on a separate thread,
 * the main thread will continuously update and wait for this to finish
 */
public class LinearSyncSegment extends StationarySegment {
    private Runnable action;

    private ExecutorService executor;

    public LinearSyncSegment(Runnable asyncAction, Pose2d pose, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("LINEAR_SYNC", 0, pose, keepPosition, markers);
        this.action = asyncAction;

        this.executor = null;
    }

    public void run() {
        executor = Executors.newSingleThreadExecutor();
        executor.submit(action);
        executor.shutdown();
    }

    public boolean isFinished() {
        if (executor == null) return false;

        return executor.isTerminated();
    }
}
