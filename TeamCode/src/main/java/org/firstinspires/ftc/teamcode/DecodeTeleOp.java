package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {

    Follower follower;
    Intake intake;
    SpinnyJeff jeff;
    Shooter shooter;
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        shooter = new Shooter(hardwareMap);

        double speedScaler = 1;

        waitForStart();

        follower.setPose(new Pose(0,0,0));
        follower.startTeleOpDrive();

        shooter.setSpeed(1);


        intake.setState(Intake.State.FORWARD);

        while (opModeIsActive()){
            // update drive
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*speedScaler,
                    -gamepad1.left_stick_x*speedScaler,
                    -gamepad1.right_stick_x*speedScaler,
                    true
            );

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
            if (gamepad1.x) shooter.engageKicker();
            else shooter.releaseKicker();

            // update turntable
            if (gamepad1.yWasPressed()) jeff.moveNext();




        }


    }

}
