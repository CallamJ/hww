package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.core.ChipBase;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@Autonomous(name = "RED Loading Zone")
public class RedLoadingZoneAuto extends ChipBase {
    Pose park = new Pose(48, 61, Math.PI);

    Pose step1 = new Pose(48, 12, Math.PI);

    private State state = State.START_MOVE_TO_HIVE;

    private Path step1() {
        return Paths.line(startPose(), step1).linear(startPose(), step1);
    }

    private Path park() {
        return Paths.line(step1, park).linear(step1, park);
    }

    @Override
    protected void onInitialize() {
        super.onInitialize();

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());
        prettyTelem.addData("Is Busy", () -> follower.isBusy());
        prettyTelem.addData("State", () -> state);

    }

    @Override
    protected Pose startPose() {
        return new Pose(63, 63, Math.PI);
    }

    @Override
    protected AllianceColor initialAllianceColor() {
        return AllianceColor.RED;
    }

    @Override
    protected void onTick() {
        super.onTick();

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

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {}

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
