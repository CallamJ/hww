package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.SwervePod;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.*;

/**
 * DifferentialPod is a hardware-backed implementation of the core `SwervePod` interface. It owns the
 * two drive motors and the PIDF controller used to
 * control pod rotation.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 * @author Callam Jomaa
 */
public class DifferentialPod implements SwervePod {
    private final DcMotorEx leftDriveMotor, rightDriveMotor;

    private final PIDFController turnPID;
    private final Pose offset;

    // Angle offset in radians applied to raw encoder angle
    private final double angleOffsetRad;

    private double motorCachingThreshold = 0.01;

    private double lastRightPower = 0;
    private double lastLeftPower = 0;

    private final double ticksPerTurnRadians;

    private DcMotor.ZeroPowerBehavior motorZPB = DcMotor.ZeroPowerBehavior.FLOAT;

    private boolean lastLeftZPBWasFloat = false, lastRightZPBWasFloat = false;

    /**
     * @param turnPIDFCoefficients PIDF coefficients for servo control
     * @param angleOffsetRad offset applied to raw encoder angle, in radians. This is the raw angle
     *                       in radians when the wheel is facing forward.
     * @param podOffset pod position offset from robot center, using the same axes as odometry pods
     */
    public DifferentialPod(HardwareMap hardwareMap, String motor1Name, String motor2Name, PIDFCoefficients turnPIDFCoefficients,
                           DcMotorSimple.Direction leftMotorDirection, DcMotorSimple.Direction rightMotorDirection,
                           double angleOffsetRad, Pose podOffset, double ticksPerTurnRadians) {

        this.leftDriveMotor = hardwareMap.get(DcMotorEx.class, motor1Name);
        this.rightDriveMotor = hardwareMap.get(DcMotorEx.class, motor2Name);

        this.turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;

        setToFloat();

        this.ticksPerTurnRadians = ticksPerTurnRadians;
        leftDriveMotor.setDirection(leftMotorDirection);
        rightDriveMotor.setDirection(rightMotorDirection);

        this.offset = podOffset;
    }

    /**
     * Returns the pod's offset from robot center.
     *
     * @return offset as a Pose
     */
    @Override
    public Pose getOffset() {
        return offset;
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    @Override
    public void setToFloat() {
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorZPB = DcMotor.ZeroPowerBehavior.FLOAT;
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    @Override
    public void setToBreak() {
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorZPB = DcMotor.ZeroPowerBehavior.BRAKE;
    }

    /**
     * Converts wheel-space theta (radians) to encoder-space theta.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        return MathFunctions.normalizeAngle(wheelTheta);
    }

    /**
     * Commands pod to a wheel heading (radians) with a drive power in [-1, 1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges if true, turn servo power is set to 0 regardless of target angle
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        // Convert hardware angle to radians and normalize
        double actualRad = getAngleAfterOffsetRad();
        actualRad = MathFunctions.normalizeAngle(actualRad);

        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double desiredRad = targetAngleRad;
        // desiredRad += Math.PI / 2.0; - removed because this is just an input offset, so theoretically it shouldnt be needed rather than just tuning rotation offset???
        desiredRad = MathFunctions.normalizeAngle(desiredRad);


        double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);

        // Minimize rotation: flip + invert drive if > 90°
        if (mag > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;
        }

        // Shortest-path error in radians (signed)
        mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
        double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;

        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(setpointRad - actualRad);
        double turnPower = ignoreAngleChanges ? 0 : MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        //get the denominator necessary to scale powers down to [-1, 1]
        double den = Math.max(Math.abs(drivePower) + Math.abs(turnPower), 1);

        double powLeft = (drivePower + turnPower) / den;
        double powRight = (drivePower - turnPower) / den;

        // if we are in brake-at-rest mode, and we have any drive signals, we don't want BRAKE zero power behavior to fight movement and cause incorrect function.
        if (Math.abs(powLeft - lastLeftPower) > motorCachingThreshold || (powLeft == 0 && lastLeftPower != 0)) {
            lastLeftPower = powLeft; // only update last power when it is actually applied, otherwise incremental changes under 0.01 would be applied to lastPower but not the motor
            leftDriveMotor.setPower(powLeft);
            if (motorZPB == DcMotor.ZeroPowerBehavior.BRAKE && powLeft == 0 && !lastLeftZPBWasFloat) {
                leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                lastLeftZPBWasFloat = true;
            } else {
                lastLeftZPBWasFloat = false;
            }
        }

        if (Math.abs(powRight - lastRightPower) > motorCachingThreshold || (powRight == 0 && lastRightPower != 0)) {
            lastRightPower = powRight; // only update last power when it is actually applied, otherwise incremental changes under 0.01 would be applied to lastPower but not the motor
            rightDriveMotor.setPower(powRight);
            if (motorZPB == DcMotor.ZeroPowerBehavior.BRAKE && powRight == 0 && !lastRightZPBWasFloat) {
                rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                lastRightZPBWasFloat = true;
            } else {
                lastRightZPBWasFloat = false;
            }
        }
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    /**
     * Returns the raw encoder angle in radians, in [0, 2pi].
     *
     * @return raw encoder angle in radians
     */
    public double getRawAngleRad() {
        double error = leftDriveMotor.getCurrentPosition() - rightDriveMotor.getCurrentPosition();
        return MathFunctions.normalizeAngle(error / ticksPerTurnRadians);
    }

    /**
     * Returns the normalized raw angle after offset, in radians.
     *
     * @return normalized angle in radians
     */
    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - angleOffsetRad;
        return MathFunctions.normalizeAngle(rad);
    }

    /**
     * Sets the drive motor caching threshold for power updates.
     *
     * @param motorCachingThreshold minimum delta before applying power update
     */
    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    /**
     * @return debug string for pod state
     */
    @Override
    public String debugString() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();
        return "diff-pod {" + "\ncurrent raw angle (rad/deg) = " + rawAngleRad + " / " + Math.toDegrees(rawAngleRad)
                + "\ncurrent angle after offset (rad/deg) = " + offsetAngleRad + " / " + Math.toDegrees(offsetAngleRad)
                + "\nleft Power = " + leftDriveMotor.getPower()
                + "\ndrive Power = " + rightDriveMotor.getPower()
                + "\n}";
    }
}
