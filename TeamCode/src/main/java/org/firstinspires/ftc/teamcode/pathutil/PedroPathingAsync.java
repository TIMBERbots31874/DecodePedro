package org.firstinspires.ftc.teamcode.pathutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class PedroPathingAsync {

    Follower follower;
    LinearOpMode opMode;

    public PedroPathingAsync(LinearOpMode opMode, Follower follower){
        this.follower = follower;
        this.opMode = opMode;
    }

    public void followPath(Path path){
        follower.followPath(path);
        waitAsync();
    }

    public void followPath(PathChain pathChain){
        follower.followPath(pathChain);
        waitAsync();
    }

    public void turnTo(double targetRadians){
        follower.turnTo(targetRadians);
        waitAsync();
    }

    public void lineToPose(Pose targetPose, double maxPower, boolean holdEnd, boolean reversed){
        Pose startPose = follower.getPose();
        double targetHeading = reversed? targetPose.getHeading() :
                AngleUnit.normalizeRadians(targetPose.getHeading() + Math.PI);
        double headingOffset = AngleUnit.normalizeRadians(targetHeading - startPose.getHeading());
        PathChain chain;
        BezierLine line = new BezierLine(startPose, targetPose);
        if (Math.abs(headingOffset) > Math.toRadians(5)){
            chain = follower.pathBuilder()
                    .addPath(line)
                    .setConstantHeadingInterpolation(targetHeading)
                    .build();
        } else {
            chain = follower.pathBuilder()
                    .addPath(line)
                    .setLinearHeadingInterpolation(startPose.getHeading(), targetHeading)
                    .build();
        }
        follower.followPath(chain, maxPower, holdEnd);
        waitAsync();
    }

    public void lineToPose(Pose targetPose, boolean reversed){
        lineToPose(targetPose, follower.getDrivetrain().getMaxPowerScaling(),
                follower.getConstants().automaticHoldEnd, reversed);
    }

    private void waitAsync(){
        while (follower.isBusy() && opMode.opModeIsActive()) follower.update();
    }

}
