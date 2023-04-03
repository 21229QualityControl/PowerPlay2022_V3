package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Supplier;

public class Cycler {
    private Roadrunner roadrunner;
    private Intake intake;
    private Outtake outtake;

    public Cycler(Roadrunner rd, Intake intk, Outtake outk) {
        this.roadrunner = rd;
        this.intake = intk;
        this.outtake = outk;
    }

    public TrajectorySequence cycle(TrajectorySequenceBuilder builder, double turretAngle, int stackLayer, Supplier<Double> runtimeSupplier) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> { // move intake out
                    intake.extenderTo(Intake.EXTENDER_BEFORE_STACK_POS);
                    intake.vslideLevel(stackLayer);
                    intake.armIntake();
                    intake.clawRelease(); // don't open fully yet
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);
                    intake.clawWide(); // open claw wide now
                })
                .waitSeconds(0.02)
                .executeSync(() -> {
                    outtake.latchBarely();
                })
                .waitSeconds(1.15)
                .executeSync(() -> { // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();
                })
                .waitSeconds(0.2)
                .executeSync(() -> {
                    intake.extendCycle();
                })
                .waitSeconds(0.2)
                .executeSync(() -> { // grab next
                    intake.clawGrab();
                })
                .waitSeconds(0.5)
                .executeSync(() -> { // lift off stack
                    intake.vslideLiftLevel(stackLayer);
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // pull back
                    intake.armStore();
                    intake.extendStore();
                })
                .waitSeconds(0.3)
                .executeSync(() -> {
                    intake.armTransfer();
                    intake.vslideTransfer();
                })
                .waitSeconds(0.35)
                .executeSync(() -> { // drop cone onto holder
                    intake.clawWide();
                })
                .waitSeconds(0.2)
                .executeSync(() -> { // move arms to collect
                    outtake.armTransferComplete();
                    intake.armStore();
                })
                .waitSeconds(0.3)
                .executeSync(() -> {
                    intake.clawClosed();
                })
                .waitSeconds(0.05)
                .build();
    }

    public TrajectorySequence sendLastCone(TrajectorySequenceBuilder builder, double turretAngle) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> { // move intake out of the way
                    intake.armStore();
                    intake.clawClosed();
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);
                })
                .waitSeconds(0.02)
                .executeSync(() -> {
                    outtake.latchBarely();
                })
                .waitSeconds(1.15)
                .executeSync(() -> { // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();
                    intake.clawWide(); // open claw wide now
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                })
                .waitSeconds(0.4)
                .build();
    }

    public TrajectorySequence grabAndTransfer(TrajectorySequenceBuilder builder, int stackLayer) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> { // grab next
                    intake.clawGrab();
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // lift off stack
                    intake.vslideLevel(stackLayer);
                })
                .waitSeconds(0.15)
                .executeSync(() -> { // pull back
                    intake.armTransfer();
                    intake.extendStore();
                })
                .waitSeconds(0.1)
                .executeSync(() -> {
                    intake.vslideDown();
                })
                .waitSeconds(0.35)
                .executeSync(() -> { // drop cone onto holder
                    intake.clawWide();
                })
                .waitSeconds(0.03)
                .executeSync(() -> { // move arms to collect
                    outtake.armTransferComplete();
                    intake.armStore();
                })
                .waitSeconds(0.1)
                .executeSync(() -> {
                    outtake.latchBarely();
                    intake.clawClosed();
                })
                .waitSeconds(0.05)
                .build();
    }
}
