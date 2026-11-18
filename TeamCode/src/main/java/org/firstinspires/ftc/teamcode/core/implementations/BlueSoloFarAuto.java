package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.core.ChipBase;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.utilities.AllianceColor;

@Autonomous(name = "BLUE Far - SOLO")
public class BlueSoloFarAuto extends ChipBase {

    ElapsedTime timer = new ElapsedTime();

    Pose step1 = new Pose(56, -12, Math.PI);

    Pose park = new Pose(-24, -61, 0);
    Pose parkControl = new Pose(-24, -36, 0);

    Pose approachGarden = new Pose(60, -52, Math.toRadians(335));

    Pose garden1 = new Pose(61,-52, Math.toRadians(335));
    Pose garden2 = new Pose(61, -61, Math.toRadians(335));

    Pose changeSides1 = new Pose(30, -12, 0);
    Pose changeSides2 = new Pose(-30, -12, 0);

    Pose lineUp = new  Pose(-48, -12, 0);

    Pose flower = new Pose(-58, -24, Math.PI);

    private State state = State.START_MOVE_TO_HIVE;

    private Path step1() {
        return Paths.line(startPose(), step1).linear(startPose(), step1);
    }

    private Path approachGarden() { return Paths.line(step1, approachGarden).linear(step1, approachGarden); }

    private Path garden1() { return Paths.line(approachGarden, garden1).linear(approachGarden, garden1); }
    private Path garden2() { return Paths.line(garden1, garden2).linear(garden1, garden2); }

    private Path changeSides1() { return Paths.line(garden2, changeSides1).linear(garden2, changeSides1); }
    private Path changeSides2() { return Paths.line(changeSides1, changeSides2).linear(changeSides1, changeSides2); }

    private Path lineUp() { return Paths.line(changeSides2, lineUp).linear(changeSides2, lineUp); }
    private Path lineUp2() { return Paths.line(flower, lineUp).linear(flower, lineUp); }

    private Path flower() { return Paths.line(lineUp, flower).linear(lineUp, flower); }

    private Path park() { return Paths.curve(lineUp, parkControl, park).linear(lineUp, park); }

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
        return new Pose(48, -63, Math.PI);
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
                timer.reset();
                break;
            case FIRING:
                if (timer.milliseconds() >= 2000) {
                    state = State.START_GARDEN_COLLECT1;
                }
                break;
            case START_APPROACH_GARDEN:
                follower.follow(approachGarden());
                state = State.APPROACH_GARDEN;
                break;
            case APPROACH_GARDEN:
                if (!follower.isBusy()) {
                    state = State.START_GARDEN_COLLECT1;
                }
                break;
            case START_GARDEN_COLLECT1:
                follower.follow(garden1());
                state = State.GARDEN_COLLECT1;
                break;
            case GARDEN_COLLECT1:
                if (!follower.isBusy()) {
                    state = State.START_GARDEN_COLLECT2;
                }
                break;
            case START_GARDEN_COLLECT2:
                follower.follow(garden2());
                state = State.GARDEN_COLLECT2;
                break;
            case GARDEN_COLLECT2:
                if (!follower.isBusy()) {
                    state = State.START_CHANGE_SIDES1;
                }
                break;
            case START_CHANGE_SIDES1:
                follower.follow(changeSides1());
                state = State.CHANGE_SIDES1;
                break;
            case CHANGE_SIDES1:
                if (!follower.isBusy()) {
                    state = State.START_CHANGE_SIDES2;
                }
                break;
            case START_CHANGE_SIDES2:
                follower.follow(changeSides2());
                state = State.CHANGE_SIDES2;
                break;
            case CHANGE_SIDES2:
                if (!follower.isBusy()) {
                    state = State.START_LINE_UP;
                }
                break;
            case START_LINE_UP:
                follower.follow(lineUp());
                state = State.LINE_UP;
                break;
            case LINE_UP:
                if (!follower.isBusy()) {
                    state = State.START_FIRING2;
                }
                break;
            case START_FIRING2:
                // fire
                state = State.FIRING2;
                timer.reset();
                break;
            case FIRING2:
                //if done firing
                if (timer.milliseconds() >= 2000) {
                    state = State.START_FLOWER_COLLECT_MOVE;
                }
                break;
            case START_FLOWER_COLLECT_MOVE:
                follower.follow(flower());
                state = State.FLOWER_COLLECT_MOVE;
                break;
            case FLOWER_COLLECT_MOVE:
                if (!follower.isBusy()) {
                    state = State.START_FLOWER_COLLECT;
                }
                break;
            case START_FLOWER_COLLECT:
                timer.reset();
                state = State.FLOWER_COLLECT;
                break;
            case FLOWER_COLLECT:
                if (timer.milliseconds() >= 3000) {
                    state = State.START_LINE_UP2;
                }
                break;
            case START_LINE_UP2:
                follower.follow(lineUp2());
                state = State.LINE_UP2;
                break;
            case LINE_UP2:
                if (!follower.isBusy()) {
                    state = State.START_FIRING3;
                }
                break;
            case START_FIRING3:
                //fire
                state = State.FIRING3;
                timer.reset();
                break;
            case FIRING3:
                if (timer.milliseconds() >= 2000) {
                    state = State.START_PARKING;
                }
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
        START_FIRING3,
        FIRING3,
        START_GARDEN_COLLECT1,
        GARDEN_COLLECT1,
        START_GARDEN_COLLECT2,
        GARDEN_COLLECT2,
        START_CHANGE_SIDES1,
        CHANGE_SIDES1,
        START_CHANGE_SIDES2,
        CHANGE_SIDES2,
        START_LINE_UP,
        LINE_UP,
        START_LINE_UP2,
        LINE_UP2,
        START_FIRING2,
        FIRING2,
        START_FLOWER_COLLECT_MOVE,
        FLOWER_COLLECT_MOVE,
        START_FLOWER_COLLECT,
        FLOWER_COLLECT,
        START_PARKING,
        PARKING,
        START_APPROACH_GARDEN,
        APPROACH_GARDEN,
        IDLE
    }

}
