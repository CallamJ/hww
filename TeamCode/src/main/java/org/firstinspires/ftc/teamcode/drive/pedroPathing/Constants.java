package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.478119917741911);
        c.yPodOffset.set(0.26134046043936665);
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
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27005234494519786);
                Controller secondaryTranslationalForward = Controller.proportional(0.09977712552569595);
                Controller primaryTranslationalLateral = Controller.proportional(0.34530495674095857);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1275809547974092);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.022747172226027214));
                c.brake.set(Controller.proportionalFeedforward(0.019335096392123132));

                c.headingFeedback.set(Controller.proportional(3.428806459484919));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0437439686586229, 0.01604389665970772));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05903394997027609, 0.0850083163978814));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0026603418199218536, 0.0016792353497279758));

                c.maxAchievableForwardVelocity.set(47.43991552499487);
                c.maxAchievableStrafeVelocity.set(40.46967617652666);
                c.naturalForwardDeceleration.set(62.43385186715687);
                c.naturalStrafeDeceleration.set(77.41532386013738);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}