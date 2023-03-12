package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.firstinspires.ftc.teamcode.main.environment.RobotConstants;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobotDetailed(Canvas canvas, Pose2d pose) {
        double[][] points = transformPolygon(RobotConstants.robotPoints, pose);
        canvas.strokePolygon(points[0], points[1]);

        Vector2d headingVector = pose.headingVec().times(RobotConstants.frontWall);
        canvas.strokeLine(
                pose.getX() + headingVector.getX()/2, pose.getY() + headingVector.getY()/2,
                pose.getX() + headingVector.getX(), pose.getY() + headingVector.getY()
        );
        canvas.fillCircle(pose.getX(), pose.getY(), 1);
    }

    private static double[][] transformPolygon(List<Vector2d> polygon, Pose2d transformation) {
        double[][] numeralPoints = new double[2][polygon.size()];
        for (int i = 0; i < polygon.size(); i++) {
            Vector2d point = polygon.get(i).rotated(transformation.getHeading()).plus(transformation.vec());
            numeralPoints[0][i] = point.getX();
            numeralPoints[1][i] = point.getY();
        }
        return numeralPoints;
    }
}
