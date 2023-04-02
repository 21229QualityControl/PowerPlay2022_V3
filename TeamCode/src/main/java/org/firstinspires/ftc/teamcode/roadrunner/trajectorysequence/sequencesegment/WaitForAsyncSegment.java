package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.ActiveLinearAsyncManager;

import java.util.List;

public class WaitForAsyncSegment extends StationarySegment {
    private String asyncName;

    public WaitForAsyncSegment(String asyncName, Pose2d pose, boolean keepPosition, List<TrajectoryMarker> markers) {
        super("WAITING_FOR_LINEAR_ASYNC_END", 0, pose, keepPosition, markers);
        this.asyncName = asyncName;
    }

    public boolean isAsyncFinished() {
        return ActiveLinearAsyncManager.isFinished(asyncName);
    }
}
