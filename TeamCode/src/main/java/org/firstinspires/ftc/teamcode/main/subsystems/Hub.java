package org.firstinspires.ftc.teamcode.main.subsystems;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

/**
 * Wrapper class for our hub.
 * Includes IMU and the builtin voltage sensor.
 *
 * Modified from Roadrunner Quickstart's SampleMecanumDrive
 */
public class Hub {
    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private double zeroHeading = 0;

    public Hub(HardwareMap hardwareMap) {
        PhotonCore.enable();
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap); // ensure v1.8.2 hub firmware

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Z);


        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double getHeading() {
        return getRawExternalHeading() - zeroHeading;
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getExternalHeadingVelocity() { // FIXME: Not returning correct value
        return imu.getAngularVelocity().zRotationRate;
    }

    public double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    public void setHeading(double newHeading) {
        zeroHeading = getRawExternalHeading() - newHeading;
    }
}
