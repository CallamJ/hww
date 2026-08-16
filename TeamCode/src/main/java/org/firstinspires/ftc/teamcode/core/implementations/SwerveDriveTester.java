package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.DifferentialPod;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;

@Configurable
@TeleOp(name = "4 - SwerveDriveTester")
public class SwerveDriveTester extends TeleOpCore {

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

        prettyTelem.addLine("Swerve Motors")
                .addData("Left Pod Left Motor", LLMotor::getCurrentPosition)
                .addData("Left Pod Right Motor", LRMotor::getCurrentPosition)
                .addData("Right Pod Left Motor", RLMotor::getCurrentPosition)
                .addData("Right Pod Right Motor", RRMotor::getCurrentPosition);

        prettyTelem.addLine("Swerve Pods")
                .addData("Left Pod Angle", leftPod::getAngle)
                .addData("Right Pod Angle", rightPod::getAngle);
    }

    @Override
    protected void onRun() {

    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        double xSig = gamepad1.leftStickX;
        double ySig = gamepad1.leftStickY;
        double angSig = gamepad1.rightStickX;

        calcPod(leftPod, xSig, ySig, angSig);
        calcPod(rightPod, xSig, ySig, angSig);
    }

    @Override
    protected void onTick() {

    }

    private void calcPod(DifferentialPod pod, double xSig, double ySig, double angSig){
        double podOffsetX = pod.getOffset().getX();
        double podOffsetY = pod.getOffset().getY();

        double vx = xSig - angSig * podOffsetY;
        double vy = ySig + angSig * podOffsetX;
        double magnitude = Math.hypot(vx, vy);
        double angleRad = Math.atan2(vy, vx);

        pod.move(angleRad, magnitude, false);
    }
}
