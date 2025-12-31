package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;

/*
 * Gamepad Mappings
 *
 * Gamepad1:
 *      left_stick: drive (translation)
 *      right_stick_x: lift "up and down"
 *      dpad down: slowmode toggle;     up: lift and servo  ;    left: Set Shooting pose    right: Shooter Speed
 *      A: intakeState REV     B: setPose;     X: Kicker;      Y: Spinner
 *      leftBumper: robotCentric (toggle);   rightBumper: intakeState FWD
 *      back: autoDrive (press and hold)
 *      trigger left: turning;        right: turning;
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
    boolean shootFast = true;
    Pose startPose;

    Pose shootingPose;

    boolean liftLocked = true;

    boolean operatingLift = false;


    boolean holdingPosition = false;
    boolean enableHold = true;
    Pose positionToHold;



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

        if (blackboard.containsKey("SHOOTING_POSE")){
            shootingPose = (Pose) blackboard.get("SHOOTING_POSE");
        } else {
            shootingPose = new Pose(0, 0, 0);
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

            if (gamepad1.dpadLeftWasPressed()) {
                shootingPose = drive.getPose();
            }
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

            if (gamepad1.dpadDownWasPressed()){
                slowMode = !slowMode;
                speedScaler = slowMode? 0.5 : 1.0;
            }

            if (autoDrive == null && gamepad1.backWasPressed()){
                autoDrive = new AutoDrive(shootingPose);
                holdingPosition = false;
            } else if (autoDrive != null){
                if (autoDrive.update() || !gamepad1.back){
                    autoDrive = null;
                    drive.setDrivePower(0,0,0);
                }
            }

            if (gamepad2.bWasPressed()) enableHold = !enableHold;
            telemetry.addData("holdingPose",holdingPosition);
            telemetry.addData("autodrive", autoDrive !=null);

            if (autoDrive == null){
                double px = -gamepad1.left_stick_y * speedScaler;
                double py = -gamepad1.left_stick_x * speedScaler;
                double pa = (gamepad1.left_trigger-gamepad1.right_trigger) * speedScaler;

                Pose vel = drive.getVelocity();
                boolean stopped = Math.hypot(vel.getX(), vel.getY()) <0.5 && Math.abs(vel.getHeading()) < .02;
                if (holdingPosition){
                    if (px != 0 || py!=0 || pa!=0 || !enableHold) holdingPosition = false;
                } else {
                    if (px==0 && py==0 && pa==0 && enableHold && stopped) holdingPosition=true;
                    positionToHold=drive.getPose();
                }
                if (holdingPosition){
                    diego.holdPose(positionToHold);
                } else {



                    if (!robotCentric) {
                        VectorF vXY = new VectorF((float) px, (float) py)
                                .multiplied(alliance == DiegoPathing.Alliance.RED ? 1 : -1);
                        VectorF vXYRobot = DiegoPathing.fieldToRobot(vXY, pose.getHeading());
                        px = vXYRobot.get(0);
                        py = vXYRobot.get(1);
                    }

                    drive.setDrivePower(px, py, pa);
                }
            }

            // Update Lift
            if (gamepad1.dpadUpWasPressed()){
                liftLocked = !liftLocked;
                if (liftLocked) lift.lockLift();
                else lift.releaseLift();
            }

            if (!gamepad1.dpad_up || Math.abs(gamepad1.right_stick_y) < 0.5){
                if (operatingLift) lift.holdPosition();
                else lift.setPower(0);
            } else if (gamepad1.right_stick_y < -0.5) {
                operatingLift = true;
                lift.setPower(1);
            } else if (gamepad1.right_stick_y > 0.5) {
                operatingLift = true;
                lift.setPower(-1);
            }

            telemetry.addData("operatingLift", operatingLift);

            //TO DO: add control of lift servo

            // update intake
            Intake.State intakeState = intake.getState();
            boolean rightBump1pressed = gamepad1.rightBumperWasPressed();
            boolean a1Pressed = gamepad1.aWasPressed();

            switch (intakeState){
                case FORWARD:
                    if (rightBump1pressed || operatingLift) intakeState = Intake.State.STOPPED;
                    else if (a1Pressed) intakeState = Intake.State.REVERSE;
                    break;
                case REVERSE:
                    if (a1Pressed || operatingLift) intakeState = Intake.State.STOPPED;
                    else if (rightBump1pressed) intakeState = Intake.State.FORWARD;
                    break;
                case STOPPED:
                    if (operatingLift) intakeState = Intake.State.STOPPED;
                    else if (rightBump1pressed) intakeState = Intake.State.FORWARD;
                    else if (a1Pressed) intakeState = Intake.State.REVERSE;
            }
            intake.setState(intakeState);

            // update shooter
            if (gamepad1.dpadRightWasPressed()){
                shootFast = !shootFast;
                shooter.setTargetSpeed(shootFast? 1050 : 875);
            }

            if (operatingLift) shooter.setSpeed(0);
            else shooter.update();

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
        Pose startPose;
        Pose targetPose;
        public AutoDrive(Pose targetPose) {
            startPose = drive.getPose();
            this.targetPose = targetPose;
        }

        public boolean update(){
            drive.updateOdometry();
            Pose pose = drive.getPose();
            double error = Math.hypot(pose.getY()- targetPose.getY(), pose.getX() - targetPose.getX());
            double turnError = Math.toDegrees(
                    AngleUnit.normalizeRadians(targetPose.getHeading() - pose.getHeading()));
            if (error < 1 && Math.abs(turnError) < 1) return true;
            diego.driveToward(startPose, targetPose, new MotionProfile(6,30,24), 1);
            return false;
        }
    }

}
