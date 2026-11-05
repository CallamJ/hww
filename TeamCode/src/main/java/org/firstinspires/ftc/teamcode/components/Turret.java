package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Configurable
public class Turret {
    PID turretPID;

    private final float kP = 0.01f, kI = 0, kD = 0.02f, kF = 0;

    private double targetPosition = 0;

    private final SmartMotor yawMotor;

    public Turret(SmartMotor yawMotor) {
        this.yawMotor = yawMotor;

        turretPID = PID.builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .setTolerance(1)
                .setDirectionalKF(true)
                .build();
    }

    private double unwrapTarget(double degrees, double currentAngle) {
        double minAngle = -360;
        double maxAngle = 360;
        
        double minK = Math.ceil((minAngle - degrees) / 360.0);
        double maxK = Math.floor((maxAngle - degrees) / 360.0);

        if (minK > maxK) {
            return Math.abs(currentAngle - minAngle) < Math.abs(currentAngle - maxAngle)
                    ? minAngle
                    : maxAngle;
        }

        double k = Math.round((currentAngle - degrees) / 360.0);
        k = Math.max(minK, Math.min(maxK, k));

        return degrees + (360.0 * k);
    }

    public void setTargetPosition(double degrees) {
        targetPosition = unwrapTarget(degrees, getCurrentPosition());
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        float ticksPerYawRevolution = 288;
        
        return (((double)yawMotor.getCurrentPosition()) / ticksPerYawRevolution)*360;
    }

    public void tick() {
        turretPID.calcPosition(targetPosition, getCurrentPosition());

        yawMotor.setPower(turretPID.result());
    }

    public void reset() {
        yawMotor.resetEncoder();
    }
}