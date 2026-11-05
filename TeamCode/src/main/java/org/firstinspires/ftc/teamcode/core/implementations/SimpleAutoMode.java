package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;

@Autonomous
public class SimpleAutoMode extends OpModeCore {

    Follower follower;

    Pose move1 = new Pose(48, 12, Math.PI);
    Pose startPose = new Pose(48, 63, Math.PI);

    private Path move() {
        return Paths.line(startPose, move1).linear(startPose, move1);
    }

    @Override
    protected void onInitialize() {

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);

        prettyTelem.addData("Pos X", () -> follower.pose().x());
        prettyTelem.addData("Pos Y", () -> follower.pose().y());
        prettyTelem.addData("Heading", () -> follower.pose().heading());

    }

    @Override
    protected void onRun() {
        follower.follow(move());
    }

    @Override
    protected void onTick() {
        follower.update();
    }

}
