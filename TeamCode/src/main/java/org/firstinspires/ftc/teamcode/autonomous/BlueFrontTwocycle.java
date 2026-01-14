package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Apriltag;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;

@Autonomous
public class BlueFrontTwocycle extends LinearOpMode {

    Motion motion;
    DiegoPathing pathing;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;

    Apriltag apriltag;

    Runnable updateShooter = ()->shooter.update();


    Pose shootingPose = new Pose(-14,7,Math.toRadians(-37.5));
    Pose shootingPose2 = new Pose(shootingPose.getX(), shootingPose.getY(), Math.PI);

    MotionProfile stdSpeed = new MotionProfile(8, 48,36);




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


        waitForStart();


        shooter.setTargetSpeed(855); // was 875 then changed to 860

        motion.setPose(new Pose(-52, 48, Math.toRadians(-37)));
        pathing.driveTo(shootingPose, // y was 3 x was-15
               stdSpeed, 1, shooter::update);

        double aprilHeading = Math.atan2(shootingPose.getY()-72, shootingPose.getX());
        pathing.turnTo(Math.toDegrees(aprilHeading), 90,8,1,shooter::update);

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive() && id == 0 && et.milliseconds() < 1000){
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

        pathing.turnTo(-47, 90, 8, 1, shooter::update);

        pathing.waitAsync(1500, shooter::update);


        double[] rightSpeeds = new double[3];
        double[] leftSpeeds = new double[3];


        rightSpeeds[0] = shooter.getRightSpeed();
        leftSpeeds[0] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[1]);
        pathing.waitAsync(1000, shooter::update);


        rightSpeeds[1] = shooter.getRightSpeed();
        leftSpeeds[1] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[2]);
        pathing.waitAsync(1000, shooter::update);

        rightSpeeds[2] = shooter.getRightSpeed();
        leftSpeeds[2] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);


        shooter.setSpeed(0);
        intake.setState(Intake.State.REVERSE);

        pathing.turnTo(180, 90, 8, 1);
        pathing.driveTo(new Pose(-24, 10, Math.toRadians(180)),
                stdSpeed, 1);
        pathing.driveTo(new Pose(-54, 10, Math.toRadians(180)),
                new MotionProfile(6, 24, 18), 1);

        shooter.setTargetSpeed(855);
        pathing.driveTo(shootingPose2,stdSpeed , 1, shooter::update);
        pathing.turnTo(-47, 90,8,1, shooter::update);
        jeff.setIndex(indices[0]);

        pathing.waitAsync(1000,shooter::update);

        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[1]);
        pathing.waitAsync(1000, shooter::update);


        rightSpeeds[1] = shooter.getRightSpeed();
        leftSpeeds[1] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);

        jeff.setIndex(indices[2]);
        pathing.waitAsync(1000, shooter::update);

        rightSpeeds[2] = shooter.getRightSpeed();
        leftSpeeds[2] = shooter.getLeftSpeed();
        shooter.engageKicker();
        pathing.waitAsync(750, shooter::update);
        shooter.releaseKicker();
        pathing.waitAsync(750, shooter::update);



        jeff.setIndex(0);

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

        intake.setState(Intake.State.STOPPED);
        blackboard.put("POSE",motion.getPose());

        blackboard.put("ALLIANCE", DiegoPathing.Alliance.BLUE);
        blackboard.put("SHOOTING_POSE", shootingPose);
    }
}
