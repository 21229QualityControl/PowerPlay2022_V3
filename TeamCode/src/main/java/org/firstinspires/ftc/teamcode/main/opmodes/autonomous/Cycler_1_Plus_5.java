package org.firstinspires.ftc.teamcode.main.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.main.subsystems.Intake;
import org.firstinspires.ftc.teamcode.main.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.main.subsystems.Roadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Supplier;

public class Cycler_1_Plus_5 {
    private Roadrunner roadrunner;
    private Intake intake;
    private Outtake outtake;

    public Cycler_1_Plus_5(Roadrunner rd, Intake intk, Outtake outk) {
        this.roadrunner = rd;
        this.intake = intk;
        this.outtake = outk;
    }

    public TrajectorySequence cycle(TrajectorySequenceBuilder builder, double turretAngle, int stackLayer, Supplier<Double> runtimeSupplier) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> {
                    // move intake out
                    intake.extenderTo(Intake.EXTENDER_BEFORE_STACK_POS);
                    intake.vslideLevel(stackLayer);
                    intake.armIntake();
                    intake.clawRelease(); // to go through gap

                    waitSeconds(0.01);
                    if (Thread.interrupted()) return;

                    // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // latch cone
                    outtake.latchBarely();
                    intake.clawWide(); // now through gap, open claw wide

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();
                    intake.extendCycle(); // extend intake out fully now

                    waitSeconds(0.25);
                    if (Thread.interrupted()) return;

                    // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();

                    // grab next
                    intake.clawGrab();

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // lift off stack
                    intake.vslideLiftLevel(stackLayer);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // start arm
                    intake.armTransferAuto();
                    if (stackLayer != 5) {
                        waitSeconds(0.05);
                    } else { // top layer receives special treatment
                        waitSeconds(0.05);
                        intake.extenderTo(intake.getExtenderTarget() - 80);
                        waitSeconds(0.10); // wait extra for top layer
                    }
                    if (Thread.interrupted()) return;

                    // pull back soon after
                    intake.extenderTo(60); // TODO: Put this into the constant

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // lower for transfer soon after
                    intake.armTransferAuto();
                    intake.vslideTransferAuto();

                    waitSeconds(0.25);
                    if (Thread.interrupted()) return;

                    // drop cone onto holder
                    intake.clawRelease();

                    waitSeconds(0.4);
                })
                .build();
    }

    public TrajectorySequence sendLastCone(TrajectorySequenceBuilder builder, double turretAngle) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> {
                    // move intake out of the way
                    intake.armStore();
                    intake.clawRelease(); // to go through gap

                    waitSeconds(0.01);
                    if (Thread.interrupted()) return;

                    // raise up
                    outtake.armTiltOut();
                    outtake.guideFlatOut();
                    outtake.raiseHigh();
                    outtake.setTurretAngle(turretAngle);

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    outtake.latchBarely();

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // drop cone
                    outtake.latchOpen();
                    outtake.guideRetractDown();

                    waitSeconds(0.25);
                    if (Thread.interrupted()) return;

                    // raise down
                    outtake.store();
                    outtake.turretCenter();
                    outtake.guideStoreUp();
                    outtake.armTransfer();

                    waitSeconds(0.4);
                })
                .build();
    }

    public TrajectorySequence grabAndTransfer(TrajectorySequenceBuilder builder, int stackLayer) {
        return builder
                .setKeepPosition(true)
                .executeSync(() -> {
                    // grab next
                    intake.clawGrab();

                    waitSeconds(0.3);
                    if (Thread.interrupted()) return;

                    // lift off stack
                    intake.vslideLiftLevel(stackLayer);

                    waitSeconds(0.2);
                    if (Thread.interrupted()) return;

                    // start arm
                    intake.armStore();

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // pull back soon after
                    intake.extendStore();

                    waitSeconds(0.2);
                    if (Thread.interrupted()) return;

                    intake.armTransferAuto();
                    intake.vslideTransferAuto();

                    waitSeconds(0.4);
                    if (Thread.interrupted()) return;

                    // drop cone onto holder
                    intake.clawRelease();

                    waitSeconds(0.1);
                    if (Thread.interrupted()) return;

                    // move claw out of the way
                    intake.armStore();

                    waitSeconds(0.01);
                })
                .build();
    }

    private void waitSeconds(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Preserve interrupted status
        }
    }
}
