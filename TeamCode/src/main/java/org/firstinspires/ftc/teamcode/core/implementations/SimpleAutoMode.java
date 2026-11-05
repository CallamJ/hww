package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.AimController;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@Autonomous
public class SimpleAutoMode extends OpModeCore {

    Follower follower;

    private Turret turret;

    private SmartMotor yawMotor;
    private SmartServo pitchServo;

    private AimController aimController;

    AllianceColor allianceColor = AllianceColor.RED;

    Pose startPose = new Pose(48, 63, Math.PI);
    Pose park = new Pose(48, 61, Math.PI);

    Pose step1 = new Pose(48, 12, Math.PI);

    private State state = State.START_MOVE_TO_HIVE;

    private Path step1() {
        return Paths.line(startPose, step1).linear(startPose, step1);
    }

    private Path park() {
        return Paths.line(step1, park).linear(step1, park);
    }

    @Override
    protected void onInitialize() {

        follower = Constants.create(hardwareMap);

        yawMotor = hardware.getMotor("yawMotor");
        pitchServo = hardware.getServo("pitchServo");

        turret = new Turret(yawMotor);
        turret.reset();

        aimController = new AimController(turret, pitchServo);

        follower.setPose(startPose);

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());
        prettyTelem.addData("Is Busy", () -> follower.isBusy());
        prettyTelem.addData("State", () -> state);

    }

    @Override
    protected void onTick() {
        follower.update();

        switch (state) {
            case START_MOVE_TO_HIVE:
                follower.follow(step1());
                state = State.MOVING_TO_HIVE;
                break;
            case MOVING_TO_HIVE:
                if (!follower.isBusy()) {
                    state = State.START_FIRING;
                }
                break;
            case START_FIRING:
                // fire
                state = State.FIRING;
                break;
            case FIRING:
                // if done firing
                state = State.START_PARKING;
                break;
            case START_PARKING:
                follower.follow(park());
                state = State.PARKING;
                break;
            case PARKING:
                if (!follower.isBusy()) {
                    state = State.IDLE;
                }
                break;
            case IDLE:
                break;
        }

        Pose currentPose = follower.pose();
        aimController.tick(currentPose, allianceColor);
    }

    private enum State {
        START_MOVE_TO_HIVE,
        MOVING_TO_HIVE,
        START_FIRING,
        FIRING,
        START_PARKING,
        PARKING,
        IDLE
    }

}
