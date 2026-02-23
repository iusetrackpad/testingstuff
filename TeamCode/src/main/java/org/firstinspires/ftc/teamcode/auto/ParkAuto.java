package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends OpMode {

    private Follower follower;
    private PathChain parkPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Starting pose — robot begins at (0, 0) facing forward
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Drive forward 24 inches to park
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(24, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(parkPath);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}
