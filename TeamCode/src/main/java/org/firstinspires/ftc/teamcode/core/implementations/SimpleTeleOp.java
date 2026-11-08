package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.ChipBase;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends ChipBase {
    private boolean fieldCentric = false;

    @Override
    protected void onInitialize() {
        super.onInitialize();

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());
    }

    @Override
    protected Pose startPose() {
        return new Pose(0, 0, 0);
    }

    @Override
    protected AllianceColor initialAllianceColor() {
        return AllianceColor.RED;
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
        super.onTick();
    }
}
