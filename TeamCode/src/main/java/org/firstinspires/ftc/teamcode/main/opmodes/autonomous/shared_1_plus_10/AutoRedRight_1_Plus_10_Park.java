package org.firstinspires.ftc.teamcode.main.opmodes.autonomous.shared_1_plus_10;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.environment.FieldConstants;
import org.firstinspires.ftc.teamcode.main.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.util.data.CPose2d;

@Config
@Autonomous(name = "🔴 ◢ Red Right Auto 1+10", group = "Auto 1+10", preselectTeleOp = "Manual Drive V2")
public class AutoRedRight_1_Plus_10_Park extends AutoBase {
    public static CPose2d CYCLE_POSITION =        new CPose2d(FieldConstants.redRightConeStack.plus(new Vector2d(-33, 0)), Math.toRadians(0));
    public static CPose2d SECOND_CYCLE_POSITION = new CPose2d(FieldConstants.redLeftConeStack.plus(new Vector2d(33, 1)), Math.toRadians(180));
    public static double TURRET_ANGLE = -49;
    public static double SECOND_TURRET_ANGLE = 49;

    @Override
    protected boolean isBlue() {
        return false;
    }

    @Override
    protected Pose2d getStartPose() {
        return FieldConstants.redRightStartingPosition;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "🔴⌊    ◢⌋ Shared 1+5 and park");
    }

    @Override
    protected void onRun() {
        Log.d("Autonomous", String.format("onRun() Start %.3f", getRuntime()));

        moveToJunction();

        for (int stack = 5; stack > 0; stack--) {
            if (getRuntime() < 27) {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Start %.3f", getRuntime()));
                follow(auto.cycle(builder(CYCLE_POSITION.asPose2d()), TURRET_ANGLE, stack, this::getRuntime));
                Log.d("Autonomous", String.format("cycle(" + stack + ") Ended %.3f", getRuntime()));
            } else {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Skipped %.3f", getRuntime()));
            }
        }

        moveToSecondStack();

        for (int stack = 5; stack > 0; stack--) {
            if (getRuntime() < 27 || (SIGNAL == 2 && getRuntime() < 28.5)) {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Start %.3f", getRuntime()));
                follow(auto.cycle(builder(SECOND_CYCLE_POSITION.asPose2d()), SECOND_TURRET_ANGLE, stack, this::getRuntime));
                Log.d("Autonomous", String.format("cycle(" + stack + ") Ended %.3f", getRuntime()));
            } else {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Skipped %.3f", getRuntime()));
            }
        }

        park();
    }

    private void moveToJunction() {
        Log.d("Autonomous", String.format("moveToJunction() Start %.3f", getRuntime()));
        follow(builder()
                .lineTo(new Vector2d(getStartPose().getX(), CYCLE_POSITION.getY()))
                .turnTo(CYCLE_POSITION.getHeading())
                .setKeepPosition(true)
                .waitSeconds(0.1)
                .waitForCondition(() -> !rr.isPositionMaintainerBusy(), 100)
                .build());

        Log.d("Autonomous", String.format("moveToJunction() Ended %.3f", getRuntime()));
    }

    private void moveToSecondStack() {
        Log.d("Autonomous", String.format("moveToSecondStack() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        outtake.store();
        intake.armStore();
        intake.extendStore();
        intake.clawClosed();
        intake.vslideLevel(3); // in case there's something in the intake
        follow(builder(CYCLE_POSITION.asPose2d())
                .lineTo(new Vector2d(SECOND_CYCLE_POSITION.getX(), SECOND_CYCLE_POSITION.getY()))
                .turnTo(SECOND_CYCLE_POSITION.getHeading())
                .setKeepPosition(true)
                .waitSeconds(0.1)
                .waitForCondition(() -> !rr.isPositionMaintainerBusy(), 100)
                .build());

        Log.d("Autonomous", String.format("moveToSecondStack() Ended %.3f", getRuntime()));
    }

    private void park() {
        Log.d("Autonomous", String.format("park() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        outtake.store();
        intake.armStore();
        intake.extendStore();
        intake.clawClosed();
        intake.vslideLevel(3); // in case there's something in the intake
        switch (SIGNAL) {
            case 1:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(5, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            default:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(4, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            case 3:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(3, 3).plus(new Vector2d(2)))
                        .build());
                break;
        }
        Log.d("Autonomous", String.format("park() Ended %.3f", getRuntime()));
    }
}