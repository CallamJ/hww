package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.core.implementations.AimingTeleOp;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

public class AimController {
    private final Turret turret;
    private final SmartServo pitchServo;

    public static float degreesPerPitch = 270;

    public AllianceColor allianceColor = AllianceColor.RED;

    public AimController(Turret turret, SmartServo pitchServo) {
        this.turret = turret;
        this.pitchServo = pitchServo;
    }

    private final Pose negRedHive = new Pose(-22, 14),
            posRedHive = new Pose(22, 14),
            negBlueHive = new Pose(-22, -14),
            posBlueHive = new Pose(22, -14);

    private Pose getTargetHive(Pose currentPose, AllianceColor allianceColor) {
        if (currentPose.x() > 0) {
            return allianceColor == AllianceColor.BLUE ? posBlueHive : posRedHive;
        } else {
            return allianceColor == AllianceColor.BLUE ? negBlueHive : negRedHive;
        }
    }

    private void updateTurret(Pose currentPose, Pose targetPose) {
        double relativeX = targetPose.x() - currentPose.x();
        double relativeY = targetPose.y() - currentPose.y();

        double bearing = Math.toDegrees(Math.atan2(relativeX, relativeY));

        double distance = Math.sqrt(Math.pow(relativeX, 2) + Math.pow(relativeY, 2));

        double pitch = Math.toDegrees(Math.atan2(60,distance));
        double yaw = bearing - 90 + Math.toDegrees(currentPose.heading());

        setTurretDirection(yaw, pitch);
    }

    public void tick(Pose currentPose, AllianceColor allianceColor) {
        updateTurret(currentPose, getTargetHive(currentPose, allianceColor));
        turret.tick();
    }

    public void setTurretDirection(double yaw, double pitch) {
        float degreesPerPitch = 270;
        
        turret.setTargetPosition(yaw);
        pitchServo.setPosition(0.5 + pitch/degreesPerPitch);
    }
}
