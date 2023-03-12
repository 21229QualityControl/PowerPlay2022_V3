package org.firstinspires.ftc.teamcode.main.environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Junction {
    public final JunctionType type;
    public final Vector2d position;

    public enum JunctionType {
        GROUND(2, 0),
        LOW(3, 13.5),
        MEDIUM(4, 23.5),
        HIGH(5, 33.5);

        public final int score;
        public final double height;

        JunctionType(int score, double height) {
            this.score = score;
            this.height = height;
        }
    }

    public Junction(JunctionType type, Vector2d position) {
        this.type = type;
        this.position = position;
    }

    public Pose2d findApproach(double botHeading, double offset) {
        return new Pose2d(position.plus(new Vector2d(offset).rotated(botHeading)), botHeading);
    }
}
