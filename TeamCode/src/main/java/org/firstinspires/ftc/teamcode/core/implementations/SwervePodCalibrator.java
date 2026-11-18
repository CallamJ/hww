package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;

import java.util.ArrayList;
import java.util.List;

/**
 * Deduces the DifferentialPod calibration constants for one pod. Run with the robot on blocks.
 * <p>
 * Encoder wiring and the closed-loop check are automatic. Motor encoders cannot tell pod rotation
 * apart from wheel spin, so the operator answers what the pod did, lines up one full turn for
 * ticksPerRadian, and points the wheel forward for forwardAngleRad. A final drive-left check
 * catches a wrong answer anywhere earlier.
 */
@TeleOp(name = "4 - SwervePodCalibrator")
public class SwervePodCalibrator extends OpMode {

    private static final String[][] POD_MOTOR_NAMES = {
            {"leftPodLeftMotor", "leftPodRightMotor"},
            {"rightPodLeftMotor", "rightPodRightMotor"}
    };
    private static final String[] POD_NAMES = {"left", "right"};

    private static final double PULSE_POWER = 0.25;
    private static final double PULSE_SECONDS = 0.5;
    private static final double SETTLE_SECONDS = 0.4;
    private static final int MIN_ENCODER_TICKS = 20;

    private static final double JOG_POWER = 0.35;
    private static final double FINE_JOG_POWER = 0.12;
    private static final double ALIGN_DRIVE_POWER = 0.15;
    private static final double STICK_DEADBAND = 0.05;

    // Verification gains are local so a bad SwerveConstants tune can't mask a bad calibration.
    private static final double VERIFY_TURN_P = 0.3;
    private static final double VERIFY_TURN_FF = 0.2;
    private static final double VERIFY_TOLERANCE_RAD = Math.toRadians(8);
    private static final double VERIFY_SETTLE_SECONDS = 0.3;
    private static final double VERIFY_TIMEOUT_SECONDS = 3.0;
    private static final double VERIFY_RUNAWAY_RAD = Math.toRadians(80);
    // Pedro frame: pi/2 = forward. 45 degree steps never trigger the 180 degree flip.
    private static final double[] VERIFY_TARGETS_RAD = {
            Math.PI / 2, 3 * Math.PI / 4, Math.PI / 2, Math.PI / 4, Math.PI / 2
    };
    private static final double LEFT_RAD = Math.PI;

    private enum Step {
        WIRING_LEFT, WIRING_RIGHT,
        OPPOSITE_PREP, OPPOSITE_ASK,
        SAME_PREP, SAME_ASK,
        TICKS_START, TICKS_END,
        FORWARD_ALIGN, ROLL_ASK,
        VERIFY, LEFT_CHECK,
        DONE, FAILED,
        PULSING
    }

    private DcMotorEx leftMotor, rightMotor;
    private int podIndex = 0;

    private Step step;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private String failure = "";
    private final List<String> notes = new ArrayList<>();

    // Pulse sub-state: runs raw powers, settles, then records the encoder deltas.
    private Step afterPulse;
    private double pulseLeftPower, pulseRightPower;
    private int pulseStartLeft, pulseStartRight;
    private int lastDeltaLeft, lastDeltaRight;
    private boolean pulseSettling;

    // Deduced constants.
    private boolean swapSteerAndDrive, steerReversed, driveReversed;
    private double ticksPerRadian = 1;
    private double forwardAngleRad;
    private int ticksStart;

    // Pod built from the constants deduced so far.
    private DifferentialPod pod;

    // Verification.
    private int verifyIndex;
    private final ElapsedTime settledTimer = new ElapsedTime();
    private boolean settled;
    private final List<String> verifyResults = new ArrayList<>();
    private boolean verifyPassed = true;

    private boolean prevA, prevB, prevX, prevY, prevDpadLeft, prevDpadRight;

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        boolean dpadLeft = gamepad1.dpad_left, dpadRight = gamepad1.dpad_right;
        if ((dpadLeft && !prevDpadLeft) || (dpadRight && !prevDpadRight)) podIndex = 1 - podIndex;
        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;

        telemetry.addLine("Put the robot on blocks. Wheels must spin freely.");
        telemetry.addData("Pod to calibrate (dpad to change)", POD_NAMES[podIndex]);
        telemetry.update();
    }

    @Override
    public void start() {
        leftMotor = hardwareMap.get(DcMotorEx.class, POD_MOTOR_NAMES[podIndex][0]);
        rightMotor = hardwareMap.get(DcMotorEx.class, POD_MOTOR_NAMES[podIndex][1]);

        // Probe in the native motor frame, which is what DifferentialPod uses.
        for (DcMotorEx motor : new DcMotorEx[]{leftMotor, rightMotor}) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        startPulse(PULSE_POWER, 0, Step.WIRING_LEFT);
    }

    @Override
    public void loop() {
        boolean a = pressed(gamepad1.a, prevA), b = pressed(gamepad1.b, prevB);
        boolean x = pressed(gamepad1.x, prevX), y = pressed(gamepad1.y, prevY);
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        telemetry.addData("Pod", POD_NAMES[podIndex]);
        telemetry.addData("Step", step == Step.PULSING ? "PULSING -> " + afterPulse : step);

        switch (step) {
            case PULSING:
                updatePulse();
                break;

            case WIRING_LEFT:
                if (checkWiring("left", lastDeltaLeft, lastDeltaRight)) {
                    startPulse(0, PULSE_POWER, Step.WIRING_RIGHT);
                }
                break;

            case WIRING_RIGHT:
                if (checkWiring("right", lastDeltaRight, lastDeltaLeft)) {
                    setStep(Step.OPPOSITE_PREP);
                }
                break;

            case OPPOSITE_PREP:
            case SAME_PREP:
                telemetry.addLine("Watch the pod from ABOVE. Press A to run a short pulse.");
                if (a) {
                    boolean opposite = step == Step.OPPOSITE_PREP;
                    startPulse(PULSE_POWER, opposite ? -PULSE_POWER : PULSE_POWER,
                            opposite ? Step.OPPOSITE_ASK : Step.SAME_ASK);
                }
                break;

            // Motors pushed opposite ways: a turn here means steering is the unswapped pattern.
            case OPPOSITE_ASK:
                askWhatHappened();
                if (a) steerDeduced(false, false);
                else if (b) steerDeduced(false, true);
                else if (x) setStep(Step.SAME_PREP);
                else if (y) setStep(Step.OPPOSITE_PREP);
                break;

            // Motors pushed the same way: a turn here means steering and drive are swapped.
            case SAME_ASK:
                askWhatHappened();
                if (a) steerDeduced(true, false);
                else if (b) steerDeduced(true, true);
                else if (x) fail("Both power patterns only spun the wheel. One motor is likely not driving "
                        + "(check power cable, gearbox, or gear mesh).");
                else if (y) setStep(Step.SAME_PREP);
                break;

            case TICKS_START:
                jog(0);
                telemetry.addLine("Line the pod up with a reference mark, then press A.");
                telemetry.addLine("Left stick X = coarse, right stick X = fine. Stick right = CCW.");
                if (a) {
                    ticksStart = pod.getEncoderDifferenceTicks();
                    setStep(Step.TICKS_END);
                }
                break;

            case TICKS_END:
                jog(0);
                telemetry.addLine("Turn the pod exactly ONE full turn, line it up again, then press A.");
                telemetry.addLine("B = restart this measurement.");
                telemetry.addData("Steering ticks so far", pod.getEncoderDifferenceTicks() - ticksStart);
                if (b) setStep(Step.TICKS_START);
                else if (a) {
                    int delta = pod.getEncoderDifferenceTicks() - ticksStart;
                    if (Math.abs(delta) < MIN_ENCODER_TICKS) {
                        fail("Steering ticks barely changed over a full turn (" + delta + ").");
                        break;
                    }
                    ticksPerRadian = Math.abs(delta) / (2 * Math.PI);
                    buildPod(0, 0);
                    setStep(Step.FORWARD_ALIGN);
                }
                break;

            case FORWARD_ALIGN:
                jog(0);
                telemetry.addLine("Point the wheel straight FRONT-TO-BACK (either way round), then press A.");
                if (a) {
                    forwardAngleRad = pod.getRawAngleRad();
                    setStep(Step.ROLL_ASK);
                }
                break;

            case ROLL_ASK:
                pod.setOpenLoopPower(ALIGN_DRIVE_POWER, 0);
                telemetry.addLine("The wheel is spinning slowly. Which way does the TOP of the wheel roll?");
                telemetry.addLine("A = toward the robot FRONT   B = toward the robot BACK");
                if (a || b) {
                    driveReversed = b;
                    buildPod(VERIFY_TURN_P, VERIFY_TURN_FF);
                    verifyIndex = 0;
                    settled = false;
                    setStep(Step.VERIFY);
                }
                break;

            case VERIFY:
                updateVerify();
                break;

            case LEFT_CHECK:
                pod.move(LEFT_RAD, ALIGN_DRIVE_POWER, false);
                telemetry.addLine("The pod should now point sideways with the TOP of the wheel");
                telemetry.addLine("rolling toward the robot's LEFT side.");
                telemetry.addLine("A = yes   B = no");
                if (a) setStep(Step.DONE);
                else if (b) fail("Drive-left check failed, so an earlier answer was wrong "
                        + "(most likely the CCW/CW answer). Rerun the calibrator.");
                break;

            case DONE:
                stopMotors();
                showResults();
                break;

            case FAILED:
                stopMotors();
                telemetry.addLine("CALIBRATION FAILED");
                telemetry.addLine(failure);
                for (String note : notes) telemetry.addLine(note);
                break;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (leftMotor != null) stopMotors();
    }

    private void askWhatHappened() {
        telemetry.addLine("What did the pod do? (top view)");
        telemetry.addLine("A = pod turned CCW   B = pod turned CW");
        telemetry.addLine("X = wheel spun, pod heading stayed put   Y = run again");
    }

    private void steerDeduced(boolean swap, boolean reversed) {
        swapSteerAndDrive = swap;
        steerReversed = reversed;
        buildPod(0, 0);
        setStep(Step.TICKS_START);
    }

    /** Builds the real pod class from the constants deduced so far, so every later step exercises it. */
    private void buildPod(double turnP, double turnFF) {
        // A fresh pod caches zero applied power, so it would never stop motors still running from before.
        stopMotors();
        pod = new DifferentialPod(hardwareMap,
                POD_MOTOR_NAMES[podIndex][0], POD_MOTOR_NAMES[podIndex][1],
                turnP, 0, 0, turnFF,
                swapSteerAndDrive, steerReversed, driveReversed,
                forwardAngleRad, Vector2D.cartesian(0, 0), ticksPerRadian);
    }

    private void startPulse(double leftPower, double rightPower, Step next) {
        pulseLeftPower = leftPower;
        pulseRightPower = rightPower;
        pulseStartLeft = leftMotor.getCurrentPosition();
        pulseStartRight = rightMotor.getCurrentPosition();
        pulseSettling = false;
        afterPulse = next;
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        setStep(Step.PULSING);
    }

    private void updatePulse() {
        telemetry.addData("Pulse power L/R", "%.2f / %.2f", pulseLeftPower, pulseRightPower);
        if (!pulseSettling && stepTimer.seconds() >= PULSE_SECONDS) {
            stopMotors();
            pulseSettling = true;
            stepTimer.reset();
        } else if (pulseSettling && stepTimer.seconds() >= SETTLE_SECONDS) {
            lastDeltaLeft = leftMotor.getCurrentPosition() - pulseStartLeft;
            lastDeltaRight = rightMotor.getCurrentPosition() - pulseStartRight;
            setStep(afterPulse);
        }
    }

    /** Checks that powering one motor moves its own encoder, in the same direction as the power. */
    private boolean checkWiring(String motor, int ownDelta, int otherDelta) {
        notes.add(String.format("%s motor pulse: own encoder %d, other encoder %d", motor, ownDelta, otherDelta));
        if (Math.abs(ownDelta) < MIN_ENCODER_TICKS && Math.abs(otherDelta) < MIN_ENCODER_TICKS) {
            fail("Powering the " + motor + " motor moved no encoder. Check its power cable and that "
                    + "its encoder cable is plugged in.");
            return false;
        }
        if (Math.abs(ownDelta) < MIN_ENCODER_TICKS) {
            fail("Powering the " + motor + " motor moved the OTHER motor's encoder. The encoder "
                    + "cables are swapped between ports.");
            return false;
        }
        if (ownDelta < 0) {
            fail("The " + motor + " encoder counts opposite to its power. That only happens with a "
                    + "non-matching encoder on that port; DifferentialPod's steering loop cannot work with it.");
            return false;
        }
        return true;
    }

    /** Open-loop steering from the sticks through the pod's calibrated mixing. */
    private void jog(double drivePower) {
        double coarse = applyDeadband(gamepad1.left_stick_x) * JOG_POWER;
        double fine = applyDeadband(gamepad1.right_stick_x) * FINE_JOG_POWER;
        pod.setOpenLoopPower(drivePower, coarse + fine);
    }

    /** Steps the pod through 45 degree moves and checks each one settles. */
    private void updateVerify() {
        double target = VERIFY_TARGETS_RAD[verifyIndex];
        pod.move(target, 0, false);
        double error = pod.getAngleError();

        telemetry.addLine("Verifying closed-loop steering. Hands clear of the pod.");
        telemetry.addData("Target / actual (deg)", "%.1f / %.1f",
                Math.toDegrees(target), Math.toDegrees(pod.getAngle()));
        telemetry.addData("Error (deg)", "%.1f", Math.toDegrees(error));

        String result = null;
        if (Math.abs(error) < VERIFY_TOLERANCE_RAD) {
            if (!settled) {
                settled = true;
                settledTimer.reset();
            } else if (settledTimer.seconds() >= VERIFY_SETTLE_SECONDS) {
                result = String.format("%.0f deg: settled, error %.1f deg, %.1f s",
                        Math.toDegrees(target), Math.toDegrees(error), stepTimer.seconds());
            }
        } else {
            settled = false;
            if (stepTimer.seconds() > 1.0 && Math.abs(error) > VERIFY_RUNAWAY_RAD) {
                result = String.format("%.0f deg: RAN AWAY (error %.1f deg). Steering loop sign is wrong.",
                        Math.toDegrees(target), Math.toDegrees(error));
                verifyPassed = false;
            } else if (stepTimer.seconds() > VERIFY_TIMEOUT_SECONDS) {
                result = String.format("%.0f deg: TIMED OUT (error %.1f deg)",
                        Math.toDegrees(target), Math.toDegrees(error));
                verifyPassed = false;
            }
        }

        if (result == null) return;
        verifyResults.add(result);
        verifyIndex++;
        settled = false;
        stepTimer.reset();
        if (!verifyPassed) {
            stopMotors();
            setStep(Step.DONE);
        } else if (verifyIndex >= VERIFY_TARGETS_RAD.length) {
            setStep(Step.LEFT_CHECK);
        }
    }

    private void showResults() {
        telemetry.addLine(verifyPassed ? "CALIBRATION PASSED" : "CALIBRATION FINISHED WITH VERIFY FAILURES");
        telemetry.addLine("SwerveConstants." + POD_NAMES[podIndex] + "Pod() values:");
        telemetry.addLine(String.format("  %b,   // swapSteerAndDrive", swapSteerAndDrive));
        telemetry.addLine(String.format("  %b,   // steerReversed", steerReversed));
        telemetry.addLine(String.format("  %b,   // driveReversed", driveReversed));
        telemetry.addLine(String.format("  %.4f, // forwardAngleRad", forwardAngleRad));
        telemetry.addLine(String.format("  ticksPerRadian: %.2f", ticksPerRadian));
        telemetry.addLine("Verify:");
        for (String result : verifyResults) telemetry.addLine("  " + result);
        telemetry.addLine("Notes:");
        for (String note : notes) telemetry.addLine("  " + note);
    }

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void fail(String reason) {
        stopMotors();
        failure = reason;
        setStep(Step.FAILED);
    }

    private void setStep(Step next) {
        step = next;
        stepTimer.reset();
    }

    private static boolean pressed(boolean current, boolean previous) {
        return current && !previous;
    }

    private static double applyDeadband(double input) {
        return Math.abs(input) > STICK_DEADBAND ? input : 0;
    }
}
