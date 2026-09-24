package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;

@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {

    Swerve drivetrain;

    @Override
    protected void onInitialize() {
        drivetrain = Constants.createDriveTrain(hardwareMap);
    }

    @Override
    protected void onRun() {
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        drivetrain.drive(new DrivePowers(-gamepad1.leftStickY, -gamepad1.leftStickX,
                -gamepad1.rightStickX), true);
    }

    @Override
    protected void onTick() {
    }
}
