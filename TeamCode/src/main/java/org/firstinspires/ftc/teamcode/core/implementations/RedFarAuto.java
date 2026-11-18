package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.core.ChipBase;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@Autonomous(name = "RED Far - TEAM")
public class RedFarAuto extends ChipBase {
    Pose step2 = new Pose(-48, 50, 0);

    Pose step1 = new Pose(-56, 12, 0);

    Pose park = new Pose(24, 61, 0);

    private State state = State.START_MOVE_TO_HIVE;

    private Path step1() {
        return Paths.line(startPose(), step1).linear(startPose(), step1);
    }

    private Path step2() {
        return Paths.line(step1, step2).linear(step1, step2);
    }

    private Path park() {
        return Paths.line(step2, park).linear(step2, park);
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
        return new Pose(-48, 63, 0);
    }

    @Override
    protected AllianceColor initialAllianceColor() {
        return AllianceColor.BLUE;
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
                state = State.START_PARKING1;
                break;
            case START_PARKING1:
                follower.follow(step2());
                state = State.PARKING1;
                break;
            case PARKING1:
                if (follower.isBusy()) {
                    state = State.START_PARKING2;
                }
                break;
            case START_PARKING2:
                follower.follow(park());
                state = State.PARKING2;
                break;
            case PARKING2:
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
        START_PARKING1,
        PARKING1,
        START_PARKING2,
        PARKING2,
        IDLE
    }

}
