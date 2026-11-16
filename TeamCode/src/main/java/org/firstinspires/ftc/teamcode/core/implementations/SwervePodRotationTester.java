package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;

/**
 * Open-loop steering test: left stick X rotates the left pod; right stick X rotates the right pod.
 * Stick right should turn the pod CCW (top view); if not, flip that pod's steerReversed.
 * {@link #ticksPerRadian} is editable from Panels Configurables while the opmode runs.
 */
@Configurable
@TeleOp(name = "4 - SwervePodRotationTester")
public class SwervePodRotationTester extends OpMode {

    private static final double DEADBAND = 0.1;

    /** Encoder difference ticks per radian of pod rotation. */
    public static double ticksPerRadian = 537.7 / (Math.PI * 6);

    private DifferentialPod leftPod;
    private DifferentialPod rightPod;

    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        leftPod = Constants.leftPod(hardwareMap);
        rightPod = Constants.rightPod(hardwareMap);

        leftPod.resetEncoders();
        rightPod.resetEncoders();
        leftPod.setToBreak();
        rightPod.setToBreak();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        double leftPower = applyDeadband(gamepad1.left_stick_x);
        double rightPower = applyDeadband(gamepad1.right_stick_x);
        leftPod.setOpenLoopPower(0, leftPower);
        rightPod.setOpenLoopPower(0, rightPower);

        double leftDifference = leftPod.getEncoderDifferenceTicks();
        double rightDifference = rightPod.getEncoderDifferenceTicks();
        double ratio = ticksPerRadian;

        telemetry.addData("Ticks per radian", ratio);
        telemetry.addData("Left pod encoder difference (ticks)", leftDifference);
        telemetry.addData("Left pod angle estimate (deg)", Math.toDegrees(leftDifference / ratio));
        telemetry.addData("Right pod encoder difference (ticks)", rightDifference);
        telemetry.addData("Right pod angle estimate (deg)", Math.toDegrees(rightDifference / ratio));
        telemetry.addLine("Point a wheel straight forward and copy its raw angle into forwardAngleRad:");
        telemetry.addData("Left pod raw angle (rad, Constants ticks)", "%.4f", leftPod.getRawAngleRad());
        telemetry.addData("Right pod raw angle (rad, Constants ticks)", "%.4f", rightPod.getRawAngleRad());
        telemetry.update();

        panelsTelemetry.addData("swerve.leftPod.encoderDifference", leftDifference);
        panelsTelemetry.addData("swerve.rightPod.encoderDifference", rightDifference);
        panelsTelemetry.update();
    }

    @Override
    public void stop() {
        leftPod.setOpenLoopPower(0, 0);
        rightPod.setOpenLoopPower(0, 0);
    }

    private static double applyDeadband(double input) {
        return Math.abs(input) > DEADBAND ? input : 0;
    }
}
