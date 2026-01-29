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
public class RedBackOnecycle extends LinearOpMode {

    Motion motion;
    DiegoPathing pathing;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;
    Apriltag apriltag;

    double shootHeadingDegrees = Constants.RED_BACK_SHOOTING_DEGREES;
    Pose shoot0 = new Pose(Constants.RED_BACK_SHOOTING_X,Constants.RED_BACK_SHOOTING_Y,Math.toRadians(-90));
    Pose shoot1 = new Pose(shoot0.getX(), shoot0.getY(), Math.toRadians(shootHeadingDegrees));

    double stdShooterSpeed = 1165;


    MotionProfile stdSpeed = new MotionProfile(8, 48, 36);

    Runnable updateShooter = () -> shooter.update();

    @Override
    public void runOpMode() throws InterruptedException {

        motion = new Motion(this);
        pathing = new DiegoPathing(motion, this);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.releaseKicker();
        jeff = new SpinnyJeff(hardwareMap);
        jeff.setIndex(3);
        apriltag = new Apriltag(hardwareMap);
        apriltag.setDecimation(1.0f);
        int id = 0;


        while (opModeInInit()) {
            motion.updateOdometry();
            Pose pose = motion.getPose();
            telemetry.addData("Pose", "%.1f  %.1f  %.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("streaming", apriltag.streaming());
            if (apriltag.streaming()) {
                id = apriltag.getObeliskID();
                telemetry.addData("id", id);
            }
            telemetry.update();
        }

        int[] indices;
        int jeffChange = 1;
        if (id == 0 || id == 22) {
            indices = new int[]{3, 4, 5};
            jeffChange = -1;
        } else if (id == 23) indices = new int[]{3, 2, 1};
        else indices = new int[]{4, 3, 2};

        jeff.setIndex(indices[0]);

        shooter.setTargetSpeed(stdShooterSpeed); //was 1000

        motion.setPose(new Pose(10, -61, Math.toRadians(-90)));
        pathing.driveTo(shoot0, stdSpeed, 1, shooter::update);
        pathing.turnTo(shootHeadingDegrees, 90, 8, 1, shooter::update);



        pathing.holdPoseAsync(1000, shoot1, shooter::update);

        double[] rightSpeeds = new double[6];
        double[] leftSpeeds = new double[6];


        rightSpeeds[0] = shooter.getRightSpeed();
        leftSpeeds[0] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[1]);
        pathing.waitAsync(1000, shooter::update);


        rightSpeeds[1] = shooter.getRightSpeed();
        leftSpeeds[1] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[2]);
        pathing.waitAsync(1000, shooter::update);

        rightSpeeds[2] = shooter.getRightSpeed();
        leftSpeeds[2] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(1000, shooter::update);
        shooter.releaseKicker();
//        pathing.waitAsync(250, shooter::update);

        shooter.setSpeed(0);

        intake.setState(Intake.State.REVERSE);

        pathing.driveTo(new Pose(16, -36, Math.toRadians(shootHeadingDegrees)),
                new MotionProfile(8, 32, 24), 1);
        pathing.turnTo(0, 90, 8, 1);

        pathing.driveTo(new Pose(58, -36, 0),
                new MotionProfile(8, 24, 18), 1);
        pathing.driveTo(new Pose(46,-24, 0), stdSpeed, 1);
        intake.setState(Intake.State.STOPPED);
        jeff.setIndex(0);

        while (opModeIsActive()) {
            motion.updateOdometry();
            Pose pose = motion.getPose();
            telemetry.addData("Pose", "X %.1f  Y %.1f  H %.1f", pose.getX(),
                    pose.getY(), Math.toDegrees(pose.getHeading()));
            for(int k = 0; k < 6; k++) {
                telemetry.addData("Speeds", "%d:  R %.1f  L %.1f", k, rightSpeeds[k], leftSpeeds[k]);
            }

            telemetry.update();
        }

        intake.setState(Intake.State.STOPPED);
        blackboard.put("ALLIANCE", DiegoPathing.Alliance.RED);
        blackboard.put("POSE", motion.getPose());
        blackboard.put("SHOOTING_POSE", shoot1);


    }
}