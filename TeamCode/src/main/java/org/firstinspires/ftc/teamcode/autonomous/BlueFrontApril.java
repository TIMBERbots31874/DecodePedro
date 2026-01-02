package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Apriltag;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;

@Autonomous
public class BlueFrontApril extends LinearOpMode {

    Motion motion;
    DiegoPathing pathing;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;

    Apriltag apriltag;

    Runnable updateShooter = ()->shooter.update();

    @Override
    public void runOpMode() throws InterruptedException {

        motion = new Motion(this);
        pathing = new DiegoPathing(motion,this);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        jeff.setIndex(0);
        apriltag = new Apriltag(hardwareMap);
        int id = 0;



        while (opModeInInit()){
            telemetry.addData("streaming",apriltag.streaming());
            if (apriltag.streaming()){
                id = apriltag.getObeliskID();
                telemetry.addData("id",id);
            }
            telemetry.update();
        }

        int[] indices;
        if (id == 0 || id == 22) indices = new int[]{0,1,2};
        else if (id == 23) indices = new int[] {0,2,1};
        else indices = new int[] {1,0,2};

        jeff.setIndex(indices[0]);

        shooter.setTargetSpeed(855); // was 875 then changed to 860

        motion.setPose(new Pose(-52, 48, Math.toRadians(-37)));
        pathing.driveTo(new Pose(-14.5, 1, Math.toRadians(-37)), // y was 3 x was-15
                new MotionProfile(6, 32, 24), 1, shooter::update);
        pathing.turnTo(-50, 90, 8, 1, shooter::update);

        pathing.waitAsync(3000, shooter::update);

        Pose actualShootingPose = motion.getPose();

        double[] rightSpeeds = new double[3];
        double[] leftSpeeds = new double[3];


        rightSpeeds[0] = shooter.getRightSpeed();
        leftSpeeds[0] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        jeff.setIndex(indices[1]);
        pathing.waitAsync(2000, shooter::update);


        rightSpeeds[1] = shooter.getRightSpeed();
        leftSpeeds[1] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        jeff.setIndex(indices[2]);
        pathing.waitAsync(2000, shooter::update);

        rightSpeeds[2] = shooter.getRightSpeed();
        leftSpeeds[2] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(1000, shooter::update);

        blackboard.put("ALLIANCE", DiegoPathing.Alliance.BLUE);
        blackboard.put("SHOOTING_POSE", motion.getPose());
        shooter.setSpeed(0);
        intake.setState(Intake.State.REVERSE);

        pathing.turnTo(180, 90, 8, 1);
        pathing.driveTo(new Pose(-24, 10, Math.toRadians(180)),
                new MotionProfile(6, 24, 18), 1);
        pathing.driveTo(new Pose(-51, 10, Math.toRadians(180)),
                new MotionProfile(6, 24, 18), 1);




        jeff.setIndex(0);

        while(opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            telemetry.addData("Pose", "X %.1f  Y %.1f  H %.1f", pose.getX(),
                    pose.getY(), Math.toDegrees(pose.getHeading()));

            telemetry.addData("actualShootingPose","X %.1f  Y %.1f  H %.1f",
                    actualShootingPose.getX(), actualShootingPose.getY(),
                    Math.toDegrees(actualShootingPose.getHeading()));
            telemetry.addData("Speeds 0", "R %.3f  L %.3f", rightSpeeds[0], leftSpeeds[0]);
            telemetry.addData("Speeds 1", "R %.3f  L %.3f", rightSpeeds[1], leftSpeeds[1]);
            telemetry.addData("Speeds 2", "R %.3f  L %.3f", rightSpeeds[2], leftSpeeds[2]);
            telemetry.update();
        }

        intake.setState(Intake.State.STOPPED);
        blackboard.put("POSE",motion.getPose());
    }
}
