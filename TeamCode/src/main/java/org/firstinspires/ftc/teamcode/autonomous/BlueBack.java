package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;
import org.firstinspires.ftc.teamcode.pathutil.PedroAsync;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueBack extends LinearOpMode {
    Follower follower;
    PedroAsync pedro;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        pedro = new PedroAsync(this, follower);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);

        PathChain line1 = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(-10, -61), new Pose(-10, -45)))
                                .setConstantHeadingInterpolation(Math.toRadians(270))
                                        .build();

        shooter.setSpeed(1);
        waitForStart();

        follower.setPose(new Pose(-10, -61, Math.toRadians(270)));

        pedro.followPath(line1);

        pedro.turnTo(Math.toRadians(300));

        shooter.engageKicker();
        sleep(300);
        shooter.releaseKicker();
        sleep(300);

        while(opModeIsActive()){
            follower.update();
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();
        }

    }
}
