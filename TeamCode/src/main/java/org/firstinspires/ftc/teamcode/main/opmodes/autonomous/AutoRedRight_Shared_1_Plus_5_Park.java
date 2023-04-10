package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.environment.FieldConstants;
import org.firstinspires.ftc.teamcode.util.data.CPose2d;

@Config
@Autonomous(name = "🔴 ◢ Red Right Auto Shared 1+5", group = "Auto Shared 1+5", preselectTeleOp = "Manual Drive")
public class AutoRedRight_Shared_1_Plus_5_Park extends AutoBase {
    public static CPose2d CYCLE_POSITION = new CPose2d(FieldConstants.redRightConeStack.plus(new Vector2d(-33, 0)), Math.toRadians(0));
    public static double TURRET_ANGLE = -49;

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
            if (getRuntime() < 23) {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Start %.3f", getRuntime()));
                follow(auto.cycle(builder(CYCLE_POSITION.asPose2d()), TURRET_ANGLE, stack, this::getRuntime));
                Log.d("Autonomous", String.format("cycle(" + stack + ") Ended %.3f", getRuntime()));
            } else {
                Log.d("Autonomous", String.format("cycle(" + stack + ") Skipped %.3f", getRuntime()));
            }
        }

        Log.d("Autonomous", String.format("cycle(last) Start %.3f", getRuntime()));
        follow(auto.sendLastCone(builder(CYCLE_POSITION.asPose2d()), TURRET_ANGLE));
        Log.d("Autonomous", String.format("cycle(last) Ended %.3f", getRuntime()));

//        park();
    }

    private void moveToJunction() {
        Log.d("Autonomous", String.format("moveToJunction() Start %.3f", getRuntime()));
        follow(builder()
                .lineTo(new Vector2d(getStartPose().getX(), CYCLE_POSITION.getY()))
                .build());
        follow(builder(CYCLE_POSITION.asPose2d())
                .setKeepPosition(true)
                .waitForCondition(() -> rr.getTrajectorySequenceRunner().isPositionMaintainerBusy(), 500)
                .build());

        Log.d("Autonomous", String.format("moveToJunction() Ended %.3f", getRuntime()));
    }

    private void park() {
        Log.d("Autonomous", String.format("park() Start %.3f", getRuntime()));
        outtake.setTurretAngle(0);
        switch (SIGNAL) {
            case 1:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(2, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            default:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(1, 3).plus(new Vector2d(0.5)))
                        .build());
                break;
            case 3:
                follow(builder()
                        .strafeTo(FieldConstants.getSquareCenter(0, 3).plus(new Vector2d(2)))
                        .build());
                break;
        }
        Log.d("Autonomous", String.format("park() Ended %.3f", getRuntime()));
    }
}
