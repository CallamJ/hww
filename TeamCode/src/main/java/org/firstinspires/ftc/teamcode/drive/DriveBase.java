package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.Pose;

public class DriveBase {

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private Follower follower;
    private double powerFactor = 1;
    public DriveBase(HardwareMap hardwareMap,  boolean startFollower) {
        if(startFollower){
            follower = Constants.createFollower(hardwareMap);
            localizer = follower.getPoseTracker().getLocalizer();
            drivetrain = follower.getDrivetrain();

            localizer.update();
        } else {
            localizer = new PinpointLocalizer(hardwareMap, Constants.createPinpointConstants());
            drivetrain = Constants.createDriveTrain(hardwareMap);
        }
    }

//    /**
//     * Moves and turns the robot using general power modifiers in each direction/axis
//     * @param x the power to move left/right with. Positive -> right, Negative -> left
//     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
//     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
//     */
//    public void moveUsingPower(double x, double y, double turn){
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the correct ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
//        double leftFront = ((y - x - turn) / denominator) * powerFactor;
//        double leftRear = ((y + x - turn) / denominator) * powerFactor;
//        double rightFront = ((y + x + turn) / denominator) * powerFactor;
//        double rightRear = ((y - x + turn) / denominator) * powerFactor;
//    }

    public Pose getPoseSimple(){
        localizer.update();
        com.pedropathing.geometry.Pose pose = localizer.getPose();
        return new Pose(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
//
//    /**
//     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
//     */
//    public void stop(){
//        setMotorPowers(0,0,0,0);
//    }
//=

    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }

    public Follower getFollower() {
        return follower;
    }
}
