package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.084933543768455);
        c.yPodOffset.set(-8.672019177534448);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LFront");
        c.frontRightName.set("RFront");
        c.backLeftName.set("LRear");
        c.backRightName.set("RRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.14927828507970803);
                Controller secondaryTranslationalForward = Controller.proportional(0.05515433754771221);
                Controller primaryTranslationalLateral = Controller.proportional(0.1647104191874161);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06085609874629255);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016954393093036223));
                c.brake.set(Controller.proportionalFeedforward(0.01441123412908079));

                c.headingFeedback.set(Controller.proportional(2.441463834573319));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.041132475992947934, 0.005117385704060208));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05781429978386808, 0.03715959780087352));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001037242614310564, 0.0016341135609148256));

                c.maxAchievableForwardVelocity.set(58.627197471404955);
                c.maxAchievableStrafeVelocity.set(51.63266580324172);
                c.naturalForwardDeceleration.set(43.829019023480804);
                c.naturalStrafeDeceleration.set(63.199545098786494);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}