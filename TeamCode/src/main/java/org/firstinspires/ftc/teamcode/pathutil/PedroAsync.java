package org.firstinspires.ftc.teamcode.pathutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class PedroAsync {

    Follower follower;
    LinearOpMode opMode;

    public PedroAsync(LinearOpMode opMode, Follower follower){
        this.follower = follower;
        this.opMode = opMode;
    }

    /**
     * Follow a Path asynchronously.
     * @param path  Path to be followed
     */
    public void followPath(Path path){
        follower.followPath(path);
        waitAsync();
    }

    /**
     * Follow a PathChain asynchronously.
     * @param pathChain     PathChain to be followed.
     * @param maxPower
     * @param holdEnd
     */
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd){
        follower.followPath(pathChain, maxPower, holdEnd);
        waitAsync();
    }

    /**
     * Follow a PathChain asynchronously.
     * @param pathChain
     */
    public void followPath(PathChain pathChain){
        followPath(pathChain, follower.getDrivetrain().getMaxPowerScaling(),
                follower.constants.isAutomaticHoldEnd());
    }

    /**
     * Turn to the specified heading, asynchronously
     * @param targetRadians
     */
    public void turnTo(double targetRadians){
        follower.turnTo(targetRadians);
        waitAsync();
    }

    public void lineTo(Pose targetPose, double maxPower, boolean holdPoint){
        Pose pose = follower.getPose();
        PathChain chain;
        if (Math.abs(AngleUnit.normalizeRadians(pose.getHeading() - targetPose.getHeading())) < 0.1) {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(pose, targetPose))
                    .setConstantHeadingInterpolation(targetPose.getHeading())
                    .build();
        } else {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(pose, targetPose))
                    .setLinearHeadingInterpolation(pose.getHeading(), targetPose.getHeading())
                    .build();
        }
        follower.followPath(chain);
        waitAsync();
    }

    public void lineTo(Pose targetPose){
        lineTo(targetPose, follower.getDrivetrain().getMaxPowerScaling(), follower.getConstants().isAutomaticHoldEnd());
    }

    public void lineToTangential(Pose targetPose, double maxPower, boolean holdEnd, boolean reversed){
        Pose pose = follower.getPose();
        PathBuilder builder = follower.pathBuilder()
                .addPath(new BezierLine(pose, targetPose))
                .setTangentHeadingInterpolation();
        if (reversed) builder.setReversed();
        PathChain chain = builder.build();
        follower.followPath(chain, maxPower, holdEnd);
        waitAsync();
    }

    public void lineToTangential(Pose targetPose, boolean reversed){
        lineToTangential(targetPose, follower.getDrivetrain().getMaxPowerScaling(),
                follower.getConstants().isAutomaticHoldEnd(), reversed);
    }

    public void followSpline(double startDir, double endDir, double maxPower, boolean holdEnd, Pose... controlPoints){
        Spline spline = new Spline(startDir, endDir, controlPoints);
        double startHeading = controlPoints[0].getHeading();
        double endHeading = controlPoints[controlPoints.length-1].getHeading();
        PathChain chain;
        if (Math.abs(AngleUnit.normalizeRadians(endHeading - startHeading)) < 0.1){
            chain = follower.pathBuilder()
                    .addPath(spline)
                    .setConstantHeadingInterpolation(endHeading)
                    .build();
        } else {
            chain = follower.pathBuilder()
                    .addPath(spline)
                    .setLinearHeadingInterpolation(startHeading, endHeading)
                    .build();
        }
        follower.followPath(chain, maxPower, holdEnd);
        waitAsync();
    }

    public void followSpline(double startDir, double endDir, Pose... controlPoints){
        followSpline(startDir, endDir, follower.getDrivetrain().getMaxPowerScaling(),
                follower.getConstants().isAutomaticHoldEnd(), controlPoints);
    }

    public void followSplineTangential(double startDir, double endDir, double maxPower,
                                       boolean holdEnd, boolean reversed, Pose... controlPoints){
        Spline spline = new Spline(startDir, endDir, controlPoints);
        PathBuilder builder = follower.pathBuilder()
                .addPath(spline)
                .setTangentHeadingInterpolation();
        if (reversed) {
            System.out.println("Setting reversed");
            builder = builder.setReversed();
        }
        PathChain chain = builder.build();
        follower.followPath(chain, maxPower, holdEnd);
        waitAsync();
    }

    public void followSplineTangential(double startDir, double endDir, boolean reversed,
                                       Pose... controlPoints){
        followSplineTangential(startDir, endDir, follower.getDrivetrain().getMaxPowerScaling(),
                follower.getConstants().isAutomaticHoldEnd(), reversed, controlPoints);
    }

    private void waitAsync(){
        while (follower.isBusy() && opMode.opModeIsActive()) follower.update();
    }

}
