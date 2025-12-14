package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;

@Autonomous
public class BlueBack extends LinearOpMode {

    Motion motion;
    DiegoPathing pathing;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;

    Runnable updateShooter = ()->shooter.update();

    @Override
    public void runOpMode() throws InterruptedException {

        motion = new Motion(this);
        pathing = new DiegoPathing(motion,this);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        jeff.setIndex(0);



        waitForStart();

        shooter.setTargetSpeed(1000);

        motion.setPose(new Pose(-10, -61, Math.toRadians(-90)));
        pathing.driveTo(new Pose(-10, -52, Math.toRadians(-90)),
                new MotionProfile(6, 24, 18), 1, shooter::update);
        pathing.turnTo(-68, 90, 8, 2, shooter::update);

        pathing.waitAsync(3000, shooter::update);

        double[] rightSpeeds = new double[3];
        double[] leftSpeeds = new double[3];


        rightSpeeds[0] = shooter.getRightSpeed();
        leftSpeeds[0] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        jeff.moveNext();
        pathing.waitAsync(2000, shooter::update);


        rightSpeeds[1] = shooter.getRightSpeed();
        leftSpeeds[1] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        jeff.moveNext();
        pathing.waitAsync(2000, shooter::update);

        rightSpeeds[2] = shooter.getRightSpeed();
        leftSpeeds[2] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        while(opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            telemetry.addData("Pose", "X %.1f  Y %.1f  H %.1f", pose.getX(),
                    pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Speeds 0", "R %.3f  L %.3f", rightSpeeds[0], leftSpeeds[0]);
            telemetry.addData("Speeds 1", "R %.3f  L %.3f", rightSpeeds[1], leftSpeeds[1]);
            telemetry.addData("Speeds 2", "R %.3f  L %.3f", rightSpeeds[2], leftSpeeds[2]);
            telemetry.update();
        }

    }
}
