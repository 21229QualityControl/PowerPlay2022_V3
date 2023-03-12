package org.firstinspires.ftc.teamcode.util.data;

import org.opencv.core.Scalar;

/**
 * Dashboard configurable color object
 */
public class Color {
    public double c1, c2, c3;

    public Color(double c1, double c2, double c3) {
        this.c1 = c1;
        this.c2 = c2;
        this.c3 = c3;
    }
    public Color(double c1, double c2) {
        this(c1, c2, 0);
    }
    public Color(double c1) {
        this(c1, 0, 0);
    }
    public Color() {
        this(0, 0, 0);
    }

    public Scalar toScalar() {
        return new Scalar(c1, c2, c3);
    }
}
