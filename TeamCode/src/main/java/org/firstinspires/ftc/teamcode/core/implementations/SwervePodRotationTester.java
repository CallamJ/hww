package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import io.github.callamj.sugar.cube.api.ConfigValue;
import io.github.callamj.sugar.cube.api.Signal;
import io.github.callamj.sugar.snap.api.HotOpMode;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;

/**
 * Open-loop steering test: left stick X rotates the left pod; right stick X rotates the right pod.
 * Stick right should turn the pod CCW (top view); if not, flip that pod's steerReversed.
 */
@TeleOp(name = "4 - SwervePodRotationTester")
public class SwervePodRotationTester extends HotOpMode {

    private static final double DEADBAND = 0.1;
    private static final double DEFAULT_TICKS_PER_RADIAN = 537.7 / (Math.PI * 6);

    private DifferentialPod leftPod;
    private DifferentialPod rightPod;

    private ConfigValue<Double> ticksPerRadian;
    private Signal<Double> leftEncoderDifferenceSignal;
    private Signal<Double> rightEncoderDifferenceSignal;

    @Override
    public void onInit() {
        leftPod = Constants.leftPod(hardwareMap);
        rightPod = Constants.rightPod(hardwareMap);

        leftPod.resetEncoders();
        rightPod.resetEncoders();
        leftPod.setToBreak();
        rightPod.setToBreak();

        ticksPerRadian = sugarCube().config()
                .doubleValue("swerve.podRotationTester.ticksPerRadian")
                .defaultValue(DEFAULT_TICKS_PER_RADIAN)
                .range(0.01, 10000.0)
                .step(1.0)
                .unit("ticks/rad")
                .description("Encoder difference ticks per radian of pod rotation")
                .group("Swerve Pod Rotation Tester")
                .persist();

        leftEncoderDifferenceSignal = sugarCube().signals().doubleValue(
                "swerve.leftPod.encoderDifference", "ticks", 50);
        rightEncoderDifferenceSignal = sugarCube().signals().doubleValue(
                "swerve.rightPod.encoderDifference", "ticks", 50);
    }

    @Override
    public void onLoop() {
        double leftPower = applyDeadband(gamepad1.left_stick_x);
        double rightPower = applyDeadband(gamepad1.right_stick_x);
        leftPod.setOpenLoopPower(0, leftPower);
        rightPod.setOpenLoopPower(0, rightPower);

        double leftDifference = leftPod.getEncoderDifferenceTicks();
        double rightDifference = rightPod.getEncoderDifferenceTicks();
        double ratio = ticksPerRadian.get();

        telemetry.addData("Ticks per radian", ratio);
        telemetry.addData("Left pod encoder difference (ticks)", leftDifference);
        telemetry.addData("Left pod angle estimate (deg)", Math.toDegrees(leftDifference / ratio));
        telemetry.addData("Right pod encoder difference (ticks)", rightDifference);
        telemetry.addData("Right pod angle estimate (deg)", Math.toDegrees(rightDifference / ratio));
        telemetry.addLine("Point a wheel straight forward and copy its raw angle into forwardAngleRad:");
        telemetry.addData("Left pod raw angle (rad, Constants ticks)", "%.4f", leftPod.getRawAngleRad());
        telemetry.addData("Right pod raw angle (rad, Constants ticks)", "%.4f", rightPod.getRawAngleRad());
        telemetry.update();

        leftEncoderDifferenceSignal.publish(leftDifference);
        rightEncoderDifferenceSignal.publish(rightDifference);
    }

    @Override
    public void onStop() {
        leftPod.setOpenLoopPower(0, 0);
        rightPod.setOpenLoopPower(0, 0);
        leftEncoderDifferenceSignal.close();
        rightEncoderDifferenceSignal.close();
        ticksPerRadian.close();
    }

    private static double applyDeadband(double input) {
        return Math.abs(input) > DEADBAND ? input : 0;
    }
}
