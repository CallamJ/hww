package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.*;
import java.util.HashMap;
import java.util.Map;

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

    // Motors always run in their native direction; the three signs below describe the pod instead.
    private final DcMotorEx leftDriveMotor, rightDriveMotor;

    // Right motor sign that makes both motors spin the wheel together (-1 when swapSteerAndDrive).
    private final int rightMotorSign;
    // Applied to both the steer command and the measured angle, so the steering loop can never invert.
    private final int steerSign;
    private final int driveSign;

    private final PIDController turnPID;
    private final double turnFeedForward;
    private final Vector2D offset;

    private double lastTargetAngleRad = 0;
    private double lastDesiredAngleRad = 0;
    private double lastAngleErrorRad = 0;
    private double lastTurnPower = 0;
    private boolean driveDirectionReversed = false;

    private static final double DIRECTION_OPTIMIZATION_HYSTERESIS_RAD = Math.toRadians(5);

    // Raw steering angle in radians when the wheel points straight forward
    private double forwardAngleRad;

    private double motorCachingThreshold = 0.01;

    private double appliedRightPower = 0;
    private double appliedLeftPower = 0;

    private final double ticksPerRadian;

    private double feedForwardDeadzoneDegrees = 5;

    private DcMotor.ZeroPowerBehavior restingZPB = DcMotor.ZeroPowerBehavior.FLOAT, appliedZPB = DcMotor.ZeroPowerBehavior.FLOAT;

    /**
     * Each calibration flag fixes exactly one symptom and changes nothing else.
     * Run {@code SwervePodCalibrator} to deduce them.
     *
     * @param turnP proportional steering gain
     * @param swapSteerAndDrive set if a steering command spins the wheel instead of turning the pod
     * @param steerReversed set if the pod turns CW (top view) when it should turn CCW
     * @param driveReversed set if the wheel rolls backward when it should roll forward
     * @param forwardAngleRad {@link #getRawAngleRad()} when the wheel points straight forward
     * @param podOffset pod position offset from robot center, using the same axes as odometry pods
     * @param ticksPerRadian steering ticks ({@link #getEncoderDifferenceTicks()}) per radian of pod rotation
     */
    public DifferentialPod(HardwareMap hardwareMap, String leftMotorName, String rightMotorName,
                           double turnP, double turnI, double turnD, double turnFeedForward,
                           boolean swapSteerAndDrive, boolean steerReversed, boolean driveReversed,
                           double forwardAngleRad, Vector2D podOffset, double ticksPerRadian) {

        this.leftDriveMotor = hardwareMap.get(DcMotorEx.class, leftMotorName);
        this.rightDriveMotor = hardwareMap.get(DcMotorEx.class, rightMotorName);

        this.turnPID = new PIDController(turnP, turnI, turnD);
        this.turnFeedForward = turnFeedForward;
        this.forwardAngleRad = forwardAngleRad;

        this.rightMotorSign = swapSteerAndDrive ? -1 : 1;
        this.steerSign = steerReversed ? -1 : 1;
        this.driveSign = driveReversed ? -1 : 1;

        setToFloat();

        this.ticksPerRadian = ticksPerRadian;
        // Direction lives in the signs above; a stale direction from another opmode would break them.
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.offset = podOffset;
    }

    /**
     * Returns the pod's offset from robot center.
     *
     * @return offset as a Pose
     */
    @Override
    public Vector2D getOffset() {
        return offset;
    }

    public void resetEncoders(){
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Returns the pod heading in Pedro's swerve frame: 0 = robot right, pi/2 = robot forward,
     * CCW (top view) positive.
     *
     * @return heading in radians
     */
    @Override
    public double getAngle() {
        return normalizeAngle(getRawAngleRad() - forwardAngleRad + Math.PI / 2);
    }

    public double getTargetAngle() {
        return lastTargetAngleRad;
    }

    public double getDesiredAngle() {
        return lastDesiredAngleRad;
    }

    public double getLeftPower() { return appliedLeftPower; }

    public double getRightPower() { return appliedRightPower; }

    public double getAngleError() {
        return lastAngleErrorRad;
    }

    public double getTurnPower() {
        return lastTurnPower;
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
        return normalizeAngle(wheelTheta);
    }

    /**
     * Commands pod to a wheel heading (radians, Pedro frame: pi/2 = forward) with a drive power in [-1, 1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges if true, turn servo power is set to 0 regardless of target angle
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        move(targetAngleRad, drivePower, ignoreAngleChanges, true);
    }

    private void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges,
                      boolean optimizeDriveDirection) {
        lastDesiredAngleRad = normalizeAngle(targetAngleRad);
        double actualRad = normalizeAngle(getAngle());

        targetAngleRad = normalizeAngle(targetAngleRad);

        double errorRad = getSignedAngleDifference(actualRad, targetAngleRad);

        // A fresh drive request can use the equivalent wheel heading and reverse
        // its drive power. A held target has already made this choice and must not
        // be optimized again. Hysteresis prevents encoder noise around 90 degrees
        // from changing the chosen representation every loop.
        if (optimizeDriveDirection && !ignoreAngleChanges) {
            double reversedTargetAngleRad = normalizeAngle(targetAngleRad + Math.PI);
            double reversedErrorRad = getSignedAngleDifference(actualRad, reversedTargetAngleRad);

            if (driveDirectionReversed) {
                if (Math.abs(errorRad) + DIRECTION_OPTIMIZATION_HYSTERESIS_RAD
                        < Math.abs(reversedErrorRad)) {
                    driveDirectionReversed = false;
                }
            } else if (Math.abs(reversedErrorRad) + DIRECTION_OPTIMIZATION_HYSTERESIS_RAD
                    < Math.abs(errorRad)) {
                driveDirectionReversed = true;
            }

            if (driveDirectionReversed) {
                targetAngleRad = reversedTargetAngleRad;
                errorRad = reversedErrorRad;
                drivePower = -drivePower;
            }
        }

        lastTargetAngleRad = targetAngleRad;

        double mag = Math.abs(errorRad);
        double feedForwardPower = mag < Math.toRadians(feedForwardDeadzoneDegrees)
                ? 0 : Math.signum(errorRad) * turnFeedForward;

        double turnPower = ignoreAngleChanges ? 0
                : Utils.clamp(turnPID.calculate(0, errorRad) + feedForwardPower, -1.0, 1.0);
        lastAngleErrorRad = errorRad;
        lastTurnPower = turnPower;

        setOpenLoopPower(drivePower, turnPower);
    }

    /**
     * Applies drive and steer power without closed-loop steering, using the calibration signs.
     * Positive steer turns the pod CCW (top view) once steerReversed is set correctly.
     */
    public void setOpenLoopPower(double drivePower, double steerPower) {
        double drive = driveSign * drivePower;
        double steer = steerSign * steerPower;

        // get the denominator necessary to scale powers down to [-1, 1]
        double den = Math.max(Math.abs(drive) + Math.abs(steer), 1);

        double powLeft = (drive + steer) / den;
        double powRight = rightMotorSign * (drive - steer) / den;


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
     * Returns the steering angle before the forward offset, in [0, 2pi). Read this with the wheel
     * pointing forward to find forwardAngleRad.
     *
     * @return raw steering angle in radians
     */
    public double getRawAngleRad() {
        return normalizeAngle(getEncoderDifferenceTicks() / ticksPerRadian);
    }

    /**
     * Returns the steering ticks: the encoder component that changes only when the pod turns.
     * Increases as the pod turns CCW (top view) once steerReversed is set correctly.
     */
    public int getEncoderDifferenceTicks() {
        return steerSign * (leftDriveMotor.getCurrentPosition()
                - rightMotorSign * rightDriveMotor.getCurrentPosition());
    }

    public double getForwardAngleRad() {
        return forwardAngleRad;
    }

    public void setForwardAngleRad(double forwardAngleRad) {
        this.forwardAngleRad = forwardAngleRad;
    }

    /**
     * Sets the drive motor caching threshold for power updates.
     *
     * @param motorCachingThreshold minimum delta before applying power update
     */
    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public double getFeedForwardDeadzoneDegrees() {
        return feedForwardDeadzoneDegrees;
    }

    public void setFeedForwardDeadzoneDegrees(double feedForwardDeadzoneDegrees) {
        this.feedForwardDeadzoneDegrees = feedForwardDeadzoneDegrees;
    }

    /**
     * @return debug string for pod state
     */
    @Override
    public String name() {
        return leftDriveMotor.getDeviceName() + "/" + rightDriveMotor.getDeviceName();
    }

    @Override
    public Map<String, Object> debug() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngle();
        Map<String, Object> values = new HashMap<>();
        values.put("rawAngleRad", rawAngleRad);
        values.put("angleRad", offsetAngleRad);
        values.put("targetAngleRad", lastTargetAngleRad);
        values.put("angleErrorRad", lastAngleErrorRad);
        values.put("turnPower", lastTurnPower);
        values.put("leftPower", leftDriveMotor.getPower());
        values.put("rightPower", rightDriveMotor.getPower());
        return values;
    }

    /**
     * Returns the signed shortest rotation from start to end.
     * Positive is counterclockwise; negative is clockwise.
     */
    private double getSignedAngleDifference(double start, double end) {
        double difference = normalizeAngle(end - start);

        if (difference > Math.PI) {
            difference -= 2.0 * Math.PI;
        }

        return difference;
    }

    private void applyZPB(DcMotor.ZeroPowerBehavior behavior){
        if(appliedZPB == behavior) return;

        leftDriveMotor.setZeroPowerBehavior(behavior);
        rightDriveMotor.setZeroPowerBehavior(behavior);
        appliedZPB = behavior;
    }

    private static double normalizeAngle(double angle) {
        double normalized = angle % (2 * Math.PI);
        return normalized < 0 ? normalized + 2 * Math.PI : normalized;
    }
}
