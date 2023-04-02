package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence;

import android.util.Log;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.teamcode.main.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.roadrunner.PositionMaintainer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StateCopyLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.IterativeAsyncMarker;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.LinearAsyncMarker;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.LinearSyncSegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.StationarySegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.WaitForAsyncSegment;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

//@Config
public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf5042";
    public static String COLOR_INACTIVE_TURN = "#7c4dff42";
    public static String COLOR_INACTIVE_WAIT = "#dd2c0042";
    public static String COLOR_INACTIVE_SYNC = "#a3a3a342";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4caf50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";
    public static String COLOR_ACTIVE_SYNC = "#a3a3a3";

    public static int POSE_HISTORY_LIMIT = 100;

    private final TrajectoryFollower follower;
    private final PositionMaintainer positionMaintainer;

    private final PIDFController turnController;

    private final NanoClock clock;
    private final ElapsedTime latencyClock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    List<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public Consumer<Canvas> dashboardConsumer = null;
    public Consumer<TelemetryPacket> packetConsumer = null;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PositionMaintainer positionMaintainer, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;
        this.positionMaintainer = positionMaintainer;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        clock = NanoClock.system();
        latencyClock = new ElapsedTime();

        Dashboard.setUp();
        Dashboard.setTelemetryTransmissionInterval(25);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
        ActiveIterativeAsyncManager.clear();
        ActiveLinearAsyncManager.clear();
    }

    public @Nullable DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        Pose2d targetPose = null;
        DriveSignal driveSignal = null;

        TelemetryPacket packet = Dashboard.packet;
        Canvas fieldOverlay = Dashboard.fieldOverlay;

        SequenceSegment currentSegment = null;

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                stop();
                return new DriveSignal();
            }

            double now = clock.seconds();
            boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

            currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

            if (isNewTransition) {
                currentSegmentStartTime = now;
                lastSegmentIndex = currentSegmentIndex;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                remainingMarkers.addAll(currentSegment.getMarkers());
                Collections.sort(remainingMarkers, (t1, t2) -> Double.compare(t1.getTime(), t2.getTime()));

                if (currentSegment instanceof LinearSyncSegment) ((LinearSyncSegment) currentSegment).run();
            }

            double deltaTime = now - currentSegmentStartTime;

            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                if (isNewTransition)
                    follower.followTrajectory(currentTrajectory);

                if (!follower.isFollowing()) {
                    currentSegmentIndex++;

                    driveSignal = new DriveSignal();

                    positionMaintainer.resetController();
                } else {
//                    Log.d("Roadrunner", String.format("Pose:%s | Vel:%s", poseEstimate, poseVelocity));
                    driveSignal = follower.update(poseEstimate, poseVelocity);
                    lastPoseError = follower.getLastError();
                }
                targetPose = currentTrajectory.get(deltaTime);

            } else if (currentSegment instanceof TurnSegment) {
                MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(poseEstimate.getHeading());
//                Log.d("TURN", String.format("%.1f -> %.1f", Math.toDegrees(targetState.getX()), Math.toDegrees(poseEstimate.getHeading())));

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                Pose2d startPose = currentSegment.getStartPose();
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                driveSignal = new DriveSignal(
                        new Pose2d(0, 0, targetOmega + correction),
                        new Pose2d(0, 0, targetAlpha)
                );

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                    driveSignal = new DriveSignal();

                    positionMaintainer.resetController();
                }

            } else if (currentSegment instanceof StationarySegment) {
                // keep position
                if (((StationarySegment) currentSegment).isKeepingPosition()) {
                    targetPose = currentSegment.getStartPose();

                    if (isNewTransition) {
                        positionMaintainer.maintainPosition(targetPose);
                    }

                    driveSignal = positionMaintainer.update(poseEstimate, poseVelocity);
                    lastPoseError = positionMaintainer.getLastError();
                } else {
                    targetPose = currentSegment.getStartPose();
                    lastPoseError = Kinematics.calculateRobotPoseError(targetPose, poseEstimate);

                    driveSignal = new DriveSignal();
                }

                // take care of different stationary segment types
                if (currentSegment instanceof WaitSegment) {
                    if (((WaitSegment) currentSegment).test()) {
                        currentSegmentIndex++;
                        Log.d("Roadrunner", "Wait segment condition met, time=" + deltaTime + ", timeout="+currentSegment.getDuration());
                    } else if (deltaTime >= currentSegment.getDuration()) {
                        currentSegmentIndex++;
                        Log.d("Roadrunner", "Wait segment expired after " + currentSegment.getDuration());
                    }

                } else if (currentSegment instanceof LinearSyncSegment) {
                    if (((LinearSyncSegment) currentSegment).isFinished()) {
                        currentSegmentIndex++;
                    }

                } else if (currentSegment instanceof WaitForAsyncSegment) {
                    if (((WaitForAsyncSegment) currentSegment).isAsyncFinished()) {
                        currentSegmentIndex++;
                    }

                } else if (currentSegment instanceof LinearAsyncMarker) {
                    ((LinearAsyncMarker) currentSegment).start();
                    currentSegmentIndex++;

                } else if (currentSegment instanceof IterativeAsyncMarker) {
                    if (!((IterativeAsyncMarker) currentSegment).isRemove()) {
                        ((IterativeAsyncMarker) currentSegment).start();
                    } else {
                        ((IterativeAsyncMarker) currentSegment).end();
                    }
                    currentSegmentIndex++;
                }
            }

            ActiveIterativeAsyncManager.runAll();

            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        poseHistory.add(poseEstimate);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        if (currentSegment != null) packet.put("mode", currentSegment.getName());
        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("ω", Math.toDegrees(poseEstimate.getHeading()));
//        Log.d("Heading", String.format("%.2f -> %.2f, error: %.2f", Math.toDegrees(poseEstimate.getHeading()), targetPose!=null?Math.toDegrees(targetPose.getHeading()):0, Math.toDegrees(lastPoseError.getHeading())));

        packet.put("xTarget", poseEstimate.getX() + getLastPoseError().getX());
        packet.put("yTarget", poseEstimate.getY() + getLastPoseError().getY());
        packet.put("ωTarget", Math.toDegrees(poseEstimate.getHeading() + getLastPoseError().getHeading()));

        packet.put("xError", getLastPoseError().getX());
        packet.put("yError", getLastPoseError().getY());
        packet.put("ωError", Math.toDegrees(getLastPoseError().getHeading()));

        packet.put("xVel", poseVelocity.getX());
        packet.put("yVel", poseVelocity.getY());
        packet.put("nVel", poseVelocity.vec().norm());
        packet.put("ωVel", poseVelocity.getHeading());

        if (driveSignal != null) { // NOTE: This vel is for the next iteration
            packet.put("xVelTarget", driveSignal.getVel().getX());
            packet.put("yVelTarget", driveSignal.getVel().getY());
            packet.put("nVelTarget", driveSignal.getVel().vec().norm());
            packet.put("ωVelTarget", driveSignal.getVel().getHeading());
        }

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate);

        if (dashboardConsumer != null) dashboardConsumer.accept(fieldOverlay);
        if (packetConsumer != null) packetConsumer.accept(packet);

        packet.put("Latency", latencyClock.milliseconds()); latencyClock.reset();
        Dashboard.sendPacket();

        if (targetPose != null) StateCopyLocalizer.pose=targetPose;
        return driveSignal;
    }

    public void stop() {
        currentTrajectorySequence = null;
        for (TrajectoryMarker marker : remainingMarkers) {
            marker.getCallback().onMarkerReached();
        }

        remainingMarkers.clear();
        ActiveIterativeAsyncManager.clear();
        ActiveLinearAsyncManager.clear();

        currentTrajectorySequence = null;
    }
    public void forceStop() {
        currentTrajectorySequence = null;

        remainingMarkers.clear();
        ActiveIterativeAsyncManager.clear();
        ActiveLinearAsyncManager.clear();

        currentTrajectorySequence = null;
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence, SequenceSegment currentSegment,
            Pose2d targetPose, Pose2d poseEstimate
    ) {
        if (sequence != null) {
            for (int i = 0; i < sequence.size(); i++) {
                SequenceSegment segment = sequence.get(i);

                if (segment == currentSegment) continue; // will be drawn later

                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof LinearSyncSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_SYNC);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 2);
                }
            }
        }

        if (currentSegment != null) { // draw current segment
            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY);

                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath());
            } else if (currentSegment instanceof TurnSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setFill(COLOR_ACTIVE_TURN);
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3);
            } else if (currentSegment instanceof WaitSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            } else if (currentSegment instanceof LinearSyncSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_SYNC);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }
        }

        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobotDetailed(fieldOverlay, targetPose);
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobotDetailed(fieldOverlay, poseEstimate);
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
