package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class SwerveConstants {
    private SwerveConstants() { }

    public static double STRAFE_POD_X_INCHES = 0.9;
    public static double FORWARD_POD_Y_INCHES = -3.25;
    public static GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PinpointConfig createPinpointConfig() {
        return new PinpointConfig(config -> {
            config.name.set("pinpoint");
            config.xPodOffset.set(STRAFE_POD_X_INCHES);
            config.yPodOffset.set(FORWARD_POD_Y_INCHES);
            config.xPodDirection.set(STRAFE_ENCODER_DIRECTION);
            config.yPodDirection.set(FORWARD_ENCODER_DIRECTION);
            config.offsetUnits.set(DistanceUnit.INCH);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.encoderResolutionUnit.set(DistanceUnit.INCH);
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        });
    }

    public static SwerveConfig swerveConfig = new SwerveConfig(config -> {
        config.voltageCompensation.set(true);
        config.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
    });

    // Conservative starting values only. Tune these on the robot before relying
    // on path following; the old 2.x PIDF constants do not map directly to Foresight.
    public static ForesightConfig foresightConfig = new ForesightConfig(config -> {
        config.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.08))
                .put(2.5, Controller.proportional(0.2)));
        config.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.08))
                .put(2.5, Controller.proportional(0.2)));
        config.coast.set(Controller.proportionalFeedforward(0.01));
        config.brake.set(Controller.proportionalFeedforward(0.01));
        config.headingFeedback.set(Controller.proportional(1.0));
        config.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.005));
        config.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
        config.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));
        config.maxAchievableForwardVelocity.set(30.0);
        config.maxAchievableStrafeVelocity.set(30.0);
        config.naturalForwardDeceleration.set(50.0);
        config.naturalStrafeDeceleration.set(50.0);
    });

    public static DifferentialPod leftPod(HardwareMap hardwareMap) {
        return new DifferentialPod(hardwareMap,
                "leftPodLeftMotor", "leftPodRightMotor",
                0.25, 0.0, 0.0, 0.2,
                true,   // swapSteerAndDrive
                true,   // steerReversed
                false,   // driveReversed
                0,      // forwardAngleRad
                Vector2D.cartesian(0, -4.5),
                537.7);
    }

    public static DifferentialPod rightPod(HardwareMap hardwareMap) {
        return new DifferentialPod(hardwareMap,
                "rightPodLeftMotor", "rightPodRightMotor",
                0.25, 0.0, 0.0, 0.3,
                true,   // swapSteerAndDrive
                true,  // steerReversed
                true,  // driveReversed
                Math.PI, // forwardAngleRad
                Vector2D.cartesian(0, 4.5),
                537.7);
    }

    public static Swerve createDriveTrain(HardwareMap hardwareMap) {
        return new Swerve(hardwareMap, swerveConfig, leftPod(hardwareMap), rightPod(hardwareMap));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, createPinpointConfig()),
                createDriveTrain(hardwareMap),
                new Foresight(foresightConfig));
    }
}
