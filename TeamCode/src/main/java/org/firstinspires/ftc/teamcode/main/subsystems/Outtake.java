package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.hardware.MotorWithPID;

@Config
public class Outtake {
    private MotorWithPID slide;
    private MotorWithPID turret;

    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.007, 0, 0.0003);
    public static int TURRET_LEFT = 0;
    public static int TURRET_RIGHT = 0;
    public static int TURRET_CENTER = 0;

    public static PIDCoefficients SLIDE_PID = new PIDCoefficients(0.015, 0, 0.0004);
    public static int SLIDE_HIGH = 0;
    public static int SLIDE_MID = 0;
    public static int SLIDE_LOW = 0;
    public static int SLIDE_STORED = 0;

    public Outtake(HardwareMap hardwareMap) {
        this.turret = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "turret"), TURRET_PID);
        this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtake"), SLIDE_PID);
    }

    public void initialize() {
        store();
        turretCenter();
    }

    public void update() {
        turret.update();
        slide.update();
    }

    // Slide movements
    public void store() {
        slide.setTargetPosition(SLIDE_STORED);
    }
    public void raiseLow() {
        slide.setTargetPosition(SLIDE_LOW);
    }
    public void raiseMid() {
        slide.setTargetPosition(SLIDE_MID);
    }
    public void raiseHigh() {
        slide.setTargetPosition(SLIDE_HIGH);
    }

    // Turret movements
    public void turretLeft() {
        turret.setTargetPosition(TURRET_LEFT);
    }
    public void turretCenter() {
        turret.setTargetPosition(TURRET_CENTER);
    }
    public void turretRight() {
        turret.setTargetPosition(TURRET_RIGHT);
    }
}
