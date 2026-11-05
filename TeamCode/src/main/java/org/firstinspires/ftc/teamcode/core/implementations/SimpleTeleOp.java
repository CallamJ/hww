package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {

    Follower follower;

    private boolean fieldCentric = false;

    @Override
    protected void onInitialize() {

        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(0, 0, 0));

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        DrivePowers powers;

        if (gamepad1.rightBumperPressed()) {
            fieldCentric = !fieldCentric;
        }

        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(
                    gamepad1.leftStickY,
                    gamepad1.leftStickX,
                    -gamepad1.rightStickX,
                    follower.pose().heading()
            );
        } else {
            powers = new DrivePowers(-gamepad1.leftStickY, -gamepad1.leftStickX, -gamepad1.rightStickX);
        }

        follower.manual(powers);
    }

    @Override
    protected void onTick() {
        follower.update();
    }
}
