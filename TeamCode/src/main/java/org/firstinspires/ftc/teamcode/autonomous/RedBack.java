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
public class RedBack extends LinearOpMode {

    Motion motion;
    DiegoPathing pathing;
    Intake intake;
    Shooter shooter;
    SpinnyJeff jeff;

    @Override
    public void runOpMode() throws InterruptedException {

        motion = new Motion(this);
        pathing = new DiegoPathing(motion,this);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        jeff.setIndex(0);



        waitForStart();

        shooter.setSpeed(1);

        motion.setPose(new Pose(10, -61, Math.toRadians(-90)));
        pathing.driveTo(new Pose(10, -49, Math.toRadians(-90)),
                new MotionProfile(6, 24, 18), 1);
        pathing.turnTo(-112, 90, 8, 2);

        pathing.waitAsync(10000);


        shooter.engageKicker();
        pathing.waitAsync(1000);
        shooter.releaseKicker();
        pathing.waitAsync(1000);

        jeff.moveNext();
        pathing.waitAsync(2000);

        shooter.engageKicker();
        pathing.waitAsync(1000);
        shooter.releaseKicker();
        pathing.waitAsync(1000);

        jeff.moveNext();
        pathing.waitAsync(2000);

        shooter.engageKicker();
        pathing.waitAsync(1000);
        shooter.releaseKicker();
        pathing.waitAsync(1000);

        while(opModeIsActive()){
            motion.updateOdometry();
            telemetry.addData("Pose", motion.getPose().toString());
            telemetry.update();
        }

    }
}
