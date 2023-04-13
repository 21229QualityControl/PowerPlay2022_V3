package org.firstinspires.ftc.teamcode.main.environment;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Arrays;
import java.util.List;

public class RobotConstants {
    // Center of robot is considered the center of the 4 wheels
    public static double leftWall = 14.0/2; // Distance from the center to the very left
    public static double rightWall = 14.0/2; // Distance from the center to the very right
    public static double frontWall = 16.0/2; // Distance from the center to the very front
    public static double rearWall = 16.0/2; // Distance from the center to the very back

    public static List<Vector2d> robotPoints = Arrays.asList(
            new Vector2d(frontWall, leftWall), // Left Front
            new Vector2d(-rearWall, leftWall), // Left Rear
            new Vector2d(-rearWall, -rightWall), // Right Rear
            new Vector2d(frontWall, -rightWall) // Right Front
    );
    public static List<Vector2d> intakePoints = Arrays.asList(
            new Vector2d(frontWall, leftWall * 2/3),
            new Vector2d(frontWall, -rightWall * 2/3)
    );

    public static Vector2d turretCenter = new Vector2d(-2, 0);
    public static Vector2d extenderEnd = new Vector2d(-3, 0);
    public static Vector2d outtakeExtended = new Vector2d(-12.8, 0);
}
