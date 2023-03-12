package org.firstinspires.ftc.teamcode.main.environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/*
 * Constants that describe the robot's environment and the field
 *
 *                   +X
 *   R-terminal       |         B-terminal
 *                    |
 *                    |
 *   B-substation     |       R-substation
 *   +Y ----------- (0,0) ----------- -Y
 *   B-substation     |       R-substation
 *                    |
 *                    |
 *   B-terminal       |         R-terminal
 *                   -X
 *
 *                 Audience
 */
@Config
public class FieldConstants {
    // Field units
    private static final double field = 141.0; // = 6*tileTabless + 5*tab = 6*22.875 + 5*0.750 = 141.000
    private static final double wall = field/2;
    private static final double tileTabbed = 24.375; // See [am-2499 Soft Tile.PDF] (https://www.andymark.com/pages/resources-files?prefix=PDF%20Drawings/)
    private static final double tileTabless = 22.875;
    private static final double tab = 0.750;
    private static final double tabHalf = tab/2;
    private static final double tapeThickness = 2.0;
    private static final double hubRadius = 18.0/2;
    private static final double carouselRadius = 15.0/2;

    // Substations
    public static final Vector2d blueSubstation = new Vector2d(0, wall);
    public static final Vector2d redSubstation = new Vector2d(0, -wall);

    // Starting Positions
    public static final Pose2d blueRightStartingPosition = new Pose2d(-tabHalf-tileTabless-tileTabbed/2, wall-RobotConstants.frontWall, Math.toRadians(90)); // blue terminal side
    public static final Pose2d blueLeftStartingPosition = new Pose2d(tabHalf+tileTabless+tileTabbed/2, wall-RobotConstants.frontWall, Math.toRadians(90));
    public static final Pose2d redLeftStartingPosition = new Pose2d(-tabHalf-tileTabless-tileTabbed/2, -wall+RobotConstants.frontWall, Math.toRadians(-90)); // red terminal side
    public static final Pose2d redRightStartingPosition = new Pose2d(tabHalf+tileTabless+tileTabbed/2, -wall+RobotConstants.frontWall, Math.toRadians(-90));

    /**
     * index goes from 0 to 5
     * View from the audience
     */
    public static Vector2d getSquareCenter(int row, int col) {
        return new Vector2d((wall-tileTabless/2) - row*(tileTabless+tab), (wall-tileTabless/2) - col*(tileTabless+tab));
    }

    /**
     * index goes from 0 to 4
     * View from the audience
     */
    public static Junction getJunction(int row, int col) { // TODO: maybe precalculate and store all the junctions somewhere and have methods to pull them out based on situation
        Junction.JunctionType junctionType;
        if (row % 2 == 0 && col % 2 == 0) junctionType = Junction.JunctionType.GROUND;
        else if (row == 2 || col == 2) junctionType = Junction.JunctionType.HIGH;
        else if (row == 0 || row == 4 || col == 0 || col == 4) junctionType = Junction.JunctionType.LOW;
        else junctionType = Junction.JunctionType.MEDIUM;

        return new Junction(junctionType, new Vector2d((wall-tileTabless-tabHalf) - row*(tileTabless+tab), (wall-tileTabless-tabHalf) - col*(tileTabless+tab)));
    }
}
