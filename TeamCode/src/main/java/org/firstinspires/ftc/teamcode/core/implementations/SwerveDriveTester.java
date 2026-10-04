package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import io.github.callamj.sugar.cube.api.Signal;
import io.github.callamj.sugar.snap.api.HotOpMode;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;

/**
 * Robot-centric swerve drive through Pedro's {@link Swerve}, the same path the Follower uses.
 * Left stick translates, right stick X turns. Pod angles are in Pedro's frame (pi/2 = forward).
 */
@TeleOp(name = "4 - SwerveDriveTester")
public class SwerveDriveTester extends HotOpMode {
    private static final double DEADBAND = 0.1;
    private DifferentialPod leftPod, rightPod;
    private Swerve swerve;
    private Signal<Double>[] podSignals;
    /**
     * FALSE: LEFT
     * TRUE: RIGHT
     */
    private boolean podSelection = false;

    @Override
    public void onInit() {
        leftPod = Constants.leftPod(hardwareMap);
        rightPod = Constants.rightPod(hardwareMap);
        leftPod.resetEncoders();
        rightPod.resetEncoders();
        swerve = new Swerve(hardwareMap, Constants.swerveConfig, leftPod, rightPod);

        String[] names = {"left.leftPower", "left.rightPower", "right.leftPower", "right.rightPower",
                "left.actualAngle", "right.actualAngle", "left.desiredAngle", "right.desiredAngle",
                "left.targetAngle", "right.targetAngle",
                "left.pidOutput", "right.pidOutput"};
        podSignals = new Signal[names.length];
        for (int i = 0; i < names.length; i++) {
            podSignals[i] = sugarCube().signals().doubleValue("swerve.pods." + names[i],
                    names[i].contains("Angle") ? "rad" : "power", 50);
        }
    }

    @Override
    public void onLoop() {
        // Pedro: forward +, strafe + = left, turn + = CCW.
        double forward = -applyDeadband(gamepad1.left_stick_y);
        double strafeLeft = -applyDeadband(gamepad1.left_stick_x);
        double turnCcw = -applyDeadband(gamepad1.right_stick_x);
        swerve.drive(new DrivePowers(forward, strafeLeft, turnCcw), true);

        double[] values = {leftPod.getLeftPower(), leftPod.getRightPower(), rightPod.getLeftPower(),
                rightPod.getRightPower(), leftPod.getAngle(), rightPod.getAngle(), leftPod.getDesiredAngle(),
                rightPod.getDesiredAngle(), leftPod.getTargetAngle(), rightPod.getTargetAngle(),
                leftPod.getTurnPower(), rightPod.getTurnPower()};
        for (int i = 0; i < values.length; i++) podSignals[i].publish(values[i]);

        if(gamepad1.dpad_left) {
            podSelection = false;
        } else if(gamepad1.dpad_right){
            podSelection = true;
        }

        if(!podSelection){ // left
            leftPod.setForwardAngleRad(leftPod.getForwardAngleRad() + (gamepad1.left_trigger - gamepad1.right_trigger) * 0.1);
        } else { // right
            rightPod.setForwardAngleRad(rightPod.getForwardAngleRad() + (gamepad1.left_trigger - gamepad1.right_trigger) * 0.1);
        }

        telemetry.addData("Forward / strafe left / turn CCW", "%.2f / %.2f / %.2f", forward, strafeLeft, turnCcw);
        telemetry.addData("Left angle actual / target (deg)", "%.1f / %.1f",
                Math.toDegrees(leftPod.getAngle()), Math.toDegrees(leftPod.getTargetAngle()));
        telemetry.addData("Right angle actual / target (deg)", "%.1f / %.1f",
                Math.toDegrees(rightPod.getAngle()), Math.toDegrees(rightPod.getTargetAngle()));
        telemetry.addData("PID output L/R", "%.3f / %.3f", leftPod.getTurnPower(), rightPod.getTurnPower());
        telemetry.update();
    }

    @Override
    public void onStop() {
        if (swerve != null) swerve.stop(false);
        if (podSignals != null) for (Signal<Double> signal : podSignals) signal.close();
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DEADBAND ? input : 0;
    }
}
