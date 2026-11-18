package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.components.AimController;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

public abstract class ChipBase extends TeleOpCore {
    // Hardware
    protected SmartMotor yawMotor;
    protected SmartServo pitchServo;

    // Subsystems
    protected Turret turret;
    protected AimController aimController;
    protected Follower follower;

    // Utilities
    protected AllianceColor allianceColor;


    @Override
    protected void onInitialize() {
        super.onInitialize();

        // Hardware
        try {
            pitchServo = hardware.getServo("pitchServo");
        } catch (Exception e) {
            prettyTelem.error("Pitch Servo failed to initialize with message: " + e.getMessage());
        }

        // Subsystems
        try {
            yawMotor = hardware.getMotor("yawMotor");
            turret = new Turret(yawMotor);
            turret.reset();
        } catch (Exception e) {
            prettyTelem.error("Turret subsystem failed with message: " + e.getMessage());
        }

        try {
            aimController = new AimController(turret, pitchServo);
        } catch (Exception e) {
            prettyTelem.error("Aim Controller subsystem failed with message: " + e.getMessage());
        }

        try {
            follower = Constants.create(hardwareMap);
            follower.setPose(startPose());
        } catch (Exception e) {
            prettyTelem.error("Follower subsystem failed to initialize with message: " + e.getMessage());
        }

        // Utilities
        allianceColor = initialAllianceColor();
    }

    protected abstract Pose startPose();
    protected abstract AllianceColor initialAllianceColor();

    @Override
    protected void onTick() {
        super.onTick();

        follower.update();
    }
}
