package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.*;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Direction;

public class Constants {
    public static double DEFAULT_MAX_POWER = 1.0;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.35);

    public static double STRAFE_POD_X_INCHES = 0.9;
    public static double FORWARD_POD_Y_INCHES = -3.25;
    public static GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;


    public static PinpointConstants createPinpointConstants() {
        return new PinpointConstants()
                .hardwareMapName("pinpoint")
                .distanceUnit(DistanceUnit.INCH)
                .strafePodX(STRAFE_POD_X_INCHES)
                .forwardPodY(FORWARD_POD_Y_INCHES)
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION)
                .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION);
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1);

    public static DifferentialPod leftPod(HardwareMap hwm){
        return new DifferentialPod(hwm,
                "leftPodLeftMotor", "leftPodRightMotor",
                new PIDFCoefficients(0, 0, 0, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                Direction.FORWARD,
                Direction.REVERSE,
                0,
                new Pose(0, 0),
                500
        );
    }

    public static DifferentialPod rightPod(HardwareMap hwm){
        return new DifferentialPod(hwm,
                "rightPodLeftMotor", "rightPodRightMotor",
                new PIDFCoefficients(0, 0, 0, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                Direction.FORWARD,
                Direction.REVERSE,
                0,
                new Pose(0, 0),
                500
        );
    }

    public static Drivetrain createDriveTrain(HardwareMap hardwareMap){
        return new SwerveBuilder(hardwareMap, swerveConstants)
                .addPod(leftPod(hardwareMap))
                .addPod(rightPod(hardwareMap))
                .build();
    }


    public static Follower createFollower(HardwareMap hardwareMap) {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, createPinpointConstants());
        return new Follower(
                followerConstants,
                localizer,
                createDriveTrain(hardwareMap)
        );
    }
}
