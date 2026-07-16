package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.DriveBase;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;

@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {
    protected DriveBase driveBase;

    @Override
    protected void onInitialize() {
        //noinspection DuplicatedCode

        try {
            driveBase = new DriveBase(hardwareMap, true);
        } catch (Exception e) {
            prettyTelem.error("Drive base failed to initialize, skipping: " + e.getMessage());
        }
    }

    @Override
    protected void onRun() {
        driveBase.getFollower().startTeleOpDrive();
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        //noinspection DuplicatedCode

        if (driveBase != null) {
            driveBase.getFollower().setTeleOpDrive(-gamepad1.leftStickY, gamepad1.leftStickX, gamepad1.rightStickX);
        }
    }

    @Override
    protected void onTick() {
        driveBase.getFollower().update();
    }
}
