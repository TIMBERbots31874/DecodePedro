package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {

    Follower follower;
    Intake intake;
    SpinnyJeff jeff;
    Shooter shooter;
    Lift lift;


    boolean robotCentric = true;

    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        shooter = new Shooter(hardwareMap);
        lift = new Lift(hardwareMap);

        boolean slowMode = false;
        double speedScaler = 1;

        waitForStart();

        follower.setPose(new Pose(0,0,0));
        follower.startTeleOpDrive();

        shooter.setSpeed(1);


        intake.setState(Intake.State.REVERSE);

        while (opModeIsActive()){
            // update drive
            follower.update();        //Added this line of code to make the robot move
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*speedScaler,
                    -gamepad1.left_stick_x*speedScaler,
                    -gamepad1.right_stick_x*speedScaler,
                    robotCentric
            );

            Pose pose = follower.getPose();
            if (gamepad1.leftBumperWasPressed()) robotCentric = !robotCentric;
            if (gamepad1.bWasPressed()) follower.setPose(new Pose(pose.getX(), pose.getY(), Math.PI));
            pose = follower.getPose();
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

            if (gamepad1.dpadDownWasPressed()){
                slowMode = !slowMode;
                speedScaler = slowMode? 0.5 : 1.0;
            }

            if (gamepad1.dpadUpWasPressed()){
                lift.togglePower();
            }

            // update intake
            Intake.State intakeState = intake.getState();
            boolean rightBump1pressed = gamepad1.rightBumperWasPressed();
            boolean a1Pressed = gamepad1.aWasPressed();

            switch (intakeState){
                case FORWARD:
                    if (rightBump1pressed) intakeState = Intake.State.STOPPED;
                    else if (a1Pressed) intakeState = Intake.State.REVERSE;
                    break;
                case REVERSE:
                    if (rightBump1pressed) intakeState = Intake.State.FORWARD;
                    else if (a1Pressed) intakeState = Intake.State.STOPPED;
                    break;
                case STOPPED:
                    if (rightBump1pressed) intakeState = Intake.State.FORWARD;
                    else if (a1Pressed) intakeState = Intake.State.REVERSE;
            }
            intake.setState(intakeState);

            // update shooter
            // ideal shooter occurs x = y= h=

            if (gamepad1.x) shooter.engageKicker();

            else shooter.releaseKicker();

            // update turntable
            if (gamepad1.yWasPressed()) jeff.moveNext();
            telemetry.addData("jeff Index", jeff.getIndex());
            telemetry.addData("robot centric", robotCentric);
            telemetry.addData("slowmode", slowMode);
            telemetry.update();

            // turn off flywheels
           // if (gamepad1.dpadUpWasPressed())



        }


    }

}
