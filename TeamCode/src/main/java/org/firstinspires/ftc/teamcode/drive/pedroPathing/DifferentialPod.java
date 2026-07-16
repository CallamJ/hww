package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.SwervePod;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.utilities.Direction;

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

    //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
    private final DcMotorEx leftDriveMotor, rightDriveMotor;

    private final PIDFController turnPID;
    private final Pose offset;

    // Angle offset in radians applied to raw encoder angle
    private double angleOffsetRad;

    private double motorCachingThreshold = 0.01;

    private double appliedRightPower = 0;
    private double appliedLeftPower = 0;

    private final double ticksPerTurnRadians;

    private double feedForwardDeadzoneDegrees = 2;

    private final Direction leftEncoderDirection, rightEncoderDirection;

    private DcMotor.ZeroPowerBehavior restingZPB = DcMotor.ZeroPowerBehavior.FLOAT, appliedZPB = DcMotor.ZeroPowerBehavior.FLOAT;

    /**
     * @param turnPIDFCoefficients PIDF coefficients for servo control
     * @param angleOffsetRad offset applied to raw encoder angle, in radians. This is the raw angle
     *                       in radians when the wheel is facing forward.
     * @param podOffset pod position offset from robot center, using the same axes as odometry pods
     */
    public DifferentialPod(HardwareMap hardwareMap, String motor1Name, String motor2Name, PIDFCoefficients turnPIDFCoefficients,
                           DcMotorSimple.Direction leftMotorDirection, DcMotorSimple.Direction rightMotorDirection,
                           Direction leftEncoderDirection, Direction rightEncoderDirection,
                           double angleOffsetRad, Pose podOffset, double ticksPerTurnRadians) {

        this.leftDriveMotor = hardwareMap.get(DcMotorEx.class, motor1Name);
        this.rightDriveMotor = hardwareMap.get(DcMotorEx.class, motor2Name);

        this.turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;
        this.leftEncoderDirection = leftEncoderDirection;
        this.rightEncoderDirection = rightEncoderDirection;

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
        restingZPB = DcMotor.ZeroPowerBehavior.FLOAT;
        applyZPB(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    @Override
    public void setToBreak() {
        restingZPB = DcMotor.ZeroPowerBehavior.BRAKE;

        if(appliedLeftPower == 0 && appliedRightPower == 0) {
            applyZPB(DcMotor.ZeroPowerBehavior.BRAKE);
        }
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
        double actualRad = MathFunctions.normalizeAngle(getAngleAfterOffsetRad());

        targetAngleRad = MathFunctions.normalizeAngle(targetAngleRad);


        double errorRad = getSignedAngleDifference(actualRad, targetAngleRad);
        double mag = Math.abs(errorRad);

        // if target angle is farther than 90 degrees we can instead turn 180 - target the other direction and reverse the drive power
        if (mag > Math.PI / 2.0) {
            targetAngleRad = MathFunctions.normalizeAngle(targetAngleRad + Math.PI);
            drivePower = -drivePower;

            errorRad = getSignedAngleDifference(actualRad, targetAngleRad);
        }


        if (mag < (feedForwardDeadzoneDegrees * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(Math.signum(errorRad));
        }
        turnPID.updateError(errorRad);

        double turnPower = ignoreAngleChanges ? 0 : MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        // get the denominator necessary to scale powers down to [-1, 1]
        double den = Math.max(Math.abs(drivePower) + Math.abs(turnPower), 1);

        double powLeft = (-drivePower + turnPower) / den;
        double powRight = (drivePower - turnPower) / den;


        // if we are trying to move, we don't want BRAKE zero power behavior to fight movement on one motor and not the other and cause incorrect function.
        if(powLeft != 0 || powRight != 0){
            applyZPB(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            applyZPB(restingZPB);

            if (appliedLeftPower != 0) {
                leftDriveMotor.setPower(0);
                appliedLeftPower = 0;
            }

            if (appliedRightPower != 0) {
                rightDriveMotor.setPower(0);
                appliedRightPower = 0;
            }

            return;
        }

        // motor caching threshold is the amount the output power must change to be worth applying to the motor
        if (Math.abs(powLeft - appliedLeftPower) > motorCachingThreshold || (powLeft == 0 && appliedLeftPower != 0)) {
            appliedLeftPower = powLeft; // only update last power when it is actually applied, otherwise incremental changes under 0.01 would be applied to lastPower but not the motor
            leftDriveMotor.setPower(powLeft);
        }
        if (Math.abs(powRight - appliedRightPower) > motorCachingThreshold || (powRight == 0 && appliedRightPower != 0)) {
            appliedRightPower = powRight; // only update last power when it is actually applied, otherwise incremental changes under 0.01 would be applied to lastPower but not the motor
            rightDriveMotor.setPower(powRight);
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
        double difference = leftDriveMotor.getCurrentPosition() * leftEncoderDirection.toSignum()
                - rightDriveMotor.getCurrentPosition() * rightEncoderDirection.toSignum();
        return MathFunctions.normalizeAngle(difference / ticksPerTurnRadians);
    }

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public void setAngleOffsetRad(double angleOffsetRad) {
        this.angleOffsetRad = angleOffsetRad;
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
     * Returns the signed shortest rotation from start to end.
     * Positive is counterclockwise; negative is clockwise.
     */
    public static double getSignedAngleDifference(double start, double end) {
        double difference = MathFunctions.normalizeAngle(end - start);

        if (difference > Math.PI) {
            difference -= 2.0 * Math.PI;
        }

        return difference;
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

    private void applyZPB(DcMotor.ZeroPowerBehavior behavior){
        if(appliedZPB == behavior) return;

        leftDriveMotor.setZeroPowerBehavior(behavior);
        rightDriveMotor.setZeroPowerBehavior(behavior);
        appliedZPB = behavior;
    }
}
