package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Static class to transition auto values into manual
 */
@Config
public class Memory {
    public static boolean IS_BLUE = true;
    public static Pose2d LAST_POSE = new Pose2d();
}
