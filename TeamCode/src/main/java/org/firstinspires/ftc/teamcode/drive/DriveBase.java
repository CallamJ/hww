package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;

public class DriveBase {
    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final Follower follower;
    private double powerFactor = 1;

    public DriveBase(HardwareMap hardwareMap, boolean startFollower) {
        if (startFollower) {
            follower = Constants.createFollower(hardwareMap);
            localizer = follower.localizer;
            drivetrain = follower.drivetrain;
            localizer.update();
        } else {
            follower = null;
            localizer = new PinpointLocalizer(hardwareMap, Constants.createPinpointConfig());
            drivetrain = Constants.createDriveTrain(hardwareMap);
        }
    }

    public org.firstinspires.ftc.teamcode.utilities.Pose getPoseSimple() {
        localizer.update();
        Pose pose = localizer.pose();
        return new org.firstinspires.ftc.teamcode.utilities.Pose(
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    public void setPowerFactor(double powerFactor) {
        this.powerFactor = powerFactor;
    }

    public double getPowerFactor() {
        return powerFactor;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Follower getFollower() {
        return follower;
    }
}
