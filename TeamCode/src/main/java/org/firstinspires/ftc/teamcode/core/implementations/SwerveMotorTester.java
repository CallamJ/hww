package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;

@TeleOp(name = "3 - SwerveMotorTester")
public class SwerveMotorTester extends TeleOpCore {

    private SmartMotor LLMotor, LRMotor, RRMotor, RLMotor;
    private DifferentialPod leftPod, rightPod;

    @Override
    protected void onInitialize() {
        leftPod = Constants.leftPod(hardwareMap);
        rightPod = Constants.rightPod(hardwareMap);

        LLMotor = hardware.getMotor("leftPodLeftMotor");
        LRMotor = hardware.getMotor("leftPodRightMotor");
        RRMotor = hardware.getMotor("rightPodRightMotor");
        RLMotor = hardware.getMotor("rightPodLeftMotor");

        LLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prettyTelem.addLine("Swerve Pods")
                .addData("Left Pod Left Motor", LLMotor::getCurrentPosition)
                .addData("Left Pod Right Motor", LRMotor::getCurrentPosition)
                .addData("Right Pod Left Motor", RLMotor::getCurrentPosition)
                .addData("Right Pod Right Motor", RRMotor::getCurrentPosition);
    }

    @Override
    protected void onRun() {

    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        double angleRad = Math.tan(gamepad1.leftStickX / gamepad1.leftStickY);
        double magnitude = Math.hypot(gamepad1.leftStickX, gamepad1.leftStickY);
        leftPod.move(angleRad, magnitude, false);
        rightPod.move(angleRad, magnitude, false);
    }

    @Override
    protected void onTick() {

    }
}
