package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;

/*
 * Gamepad Mappings
 *
 * Gamepad1:
 *      left_stick: drive (translation)
 *      right_stick_x: drive (turning)
 *      dpad down: slowmode toggle;     up: lift toggle;    left: UNUSED    right: UNUSED
 *      A: intakeState REV     B: setPose;     X: Kicker;      Y: Spinner
 *      leftBumper: robotCentric (toggle);   rightBumper: intakeState FWD
 *      back: autoDrive (press and hold)
 *      trigger left: UNUSED;        right: UNUSED;
 */

@TeleOp
public class DecodeTeleOp extends LinearOpMode {

    Motion drive;
    DiegoPathing diego;
    Intake intake;
    SpinnyJeff jeff;
    Shooter shooter;
    Lift lift;
    boolean robotCentric = true;
    boolean slowMode = false;
    double speedScaler = 1;
    AutoDrive autoDrive = null;
    Pose startPose;
    DiegoPathing.Alliance alliance = DiegoPathing.Alliance.BLUE;


    @Override
    public void runOpMode(){
        drive = new Motion(this);
        diego = new DiegoPathing(drive, this);
        intake = new Intake(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        shooter = new Shooter(hardwareMap);
        lift = new Lift(hardwareMap);

        if (blackboard.containsKey("POSE")){
            startPose = (Pose)blackboard.get("POSE");
        } else {
            startPose = new Pose(0,0,0);
        }

        if (blackboard.containsKey("ALLIANCE")){
            alliance = (DiegoPathing.Alliance) blackboard.get("ALLIANCE");
        }

        waitForStart();

        drive.setPose(startPose);

        shooter.setTargetSpeed(1050);

        intake.setState(Intake.State.REVERSE);

        while (opModeIsActive()){
            // update drive

            drive.updateOdometry();
            Pose pose = drive.getPose();
            if (gamepad1.leftBumperWasPressed()) robotCentric = !robotCentric;
            if (gamepad1.bWasPressed()) {
                pose = new Pose(alliance == DiegoPathing.Alliance.BLUE? -10 : 10, -61,
                        Math.toRadians(-90));
                drive.setPose(pose);
            }
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

            if (gamepad1.dpadDownWasPressed()){
                slowMode = !slowMode;
                speedScaler = slowMode? 0.5 : 1.0;
            }

            if (autoDrive == null && gamepad1.backWasPressed()){
                autoDrive = new AutoDrive(startPose);
            } else if (autoDrive != null){
                if (autoDrive.update() || !gamepad1.back){
                    autoDrive = null;
                    drive.setDrivePower(0,0,0);
                }
            }

            if (autoDrive == null){
                double px = -gamepad1.left_stick_y * speedScaler;
                double py = -gamepad1.left_stick_x * speedScaler;
                double pa = -gamepad1.right_stick_x * speedScaler;

                if (!robotCentric){
                    VectorF vXY = new VectorF((float)px,(float)py)
                            .multiplied(alliance== DiegoPathing.Alliance.RED? 1 : -1);
                    VectorF vXYRobot = DiegoPathing.fieldToRobot(vXY, pose.getHeading());
                    px = vXYRobot.get(0);
                    py = vXYRobot.get(1);
                }

                drive.setDrivePower(px, py, pa);
            }

            // Update Lift

            if (gamepad1.dpadUpWasPressed()){
                lift.togglePower();
            }

            //TO DO: add control of lift servo

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

            shooter.update();

            if (gamepad1.x) shooter.engageKicker();
            else shooter.releaseKicker();

            telemetry.addData("shooter speeds", "R %.2f  L %.2f",
                    shooter.getRightSpeed(), shooter.getLeftSpeed());
            telemetry.addData("shooter powers", "R %.3f  L %.3f",
                    shooter.getRightPower(), shooter.getLeftPower());


            // update turntable
            if (gamepad1.yWasPressed()) jeff.moveNext();

            telemetry.addData("jeff Index", jeff.getIndex());
            telemetry.addData("robot centric", robotCentric);
            telemetry.addData("slowmode", slowMode);
            telemetry.update();

        }
    }

    class AutoDrive{
        public AutoDrive(Pose targetPose){}
        public boolean update(){ return true; }
    }

}
