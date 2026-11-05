package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.AimController;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@TeleOp(name = "1 - Aiming TeleOp")
public class AimingTeleOp extends TeleOpCore {

    Follower follower;

    private Turret turret;

    private SmartMotor yawMotor;
    private SmartServo pitchServo;

    private boolean fieldCentric = false;

    private AllianceColor allianceColor;

    private AimController aimController;

    @Override
    protected void onInitialize() {

        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(0, 0, 0));

        yawMotor = hardware.getMotor("yawMotor");
        pitchServo = hardware.getServo("pitchServo");

        turret = new Turret(yawMotor);
        turret.reset();

        aimController = new AimController(turret, pitchServo);

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());

        prettyTelem.addData("Target", turret::getTargetPosition);
        prettyTelem.addData("Current", turret::getCurrentPosition);
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        DrivePowers powers;

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

       if (gamepad1.aPressed()) {
           if (allianceColor == AllianceColor.BLUE) {
               allianceColor = AllianceColor.RED;
           } else {
               allianceColor = AllianceColor.BLUE;
           }
       }

        if (gamepad1.rightBumperPressed()) {
            fieldCentric = !fieldCentric;
        }

        follower.manual(powers);
    }



    @Override
    protected void onTick() {
        follower.update();
        Pose currentPose = follower.pose();
        aimController.tick(currentPose, allianceColor);
    }
}
