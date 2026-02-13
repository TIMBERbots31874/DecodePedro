package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Apriltag;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.SpinnyJeff;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;

import java.util.List;

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
 *
 * gamepad2
 *
 *      a: toggle aliance  b: reset pose   x: toggle hold pose  y: spinner
 *      dpad: left: set shooting pose     dpad down: toggle slow mode
 *      left bumper: robot centric  right bumper: toggle cubicMode
 *      back: turn to target
 *
 */

@TeleOp
public class DecodeTeleOp extends LinearOpMode {

    Motion drive;
    DiegoPathing diego;
    Intake intake;
    SpinnyJeff jeff;
    Shooter shooter;
    Lift lift;
    Apriltag apriltag;
    boolean robotCentric = true;
    boolean slowMode = false;
    boolean cubicMode = false;
    double speedScaler = 1;


    AutoDrive autoDrive = null;
    boolean shootFast = false;
    double fastShootSpeed = 1175;
    double slowShootSpeed = 855;
    Pose startPose;

    Pose shootingPose;

    boolean liftLocked = true;

    boolean operatingLift = false;


    boolean holdingPosition = false;
    boolean enableHold = true;
    Pose positionToHold;

//    boolean setSTDShootingSpeed = 290;




    DiegoPathing.Alliance alliance = DiegoPathing.Alliance.BLUE;


    @Override
    public void runOpMode(){
        drive = new Motion(this);
        diego = new DiegoPathing(drive, this);
        intake = new Intake(hardwareMap);
        jeff = new SpinnyJeff(hardwareMap);
        shooter = new Shooter(hardwareMap);
        lift = new Lift(hardwareMap);
        apriltag = new Apriltag(hardwareMap);

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

        jeff.setIndex(0);
        shooter.releaseKicker();

        shooter.setTargetSpeed(slowShootSpeed);

        intake.setState(Intake.State.REVERSE);

        while (opModeIsActive()){
            // update alliance
            if (gamepad2.aWasPressed()){
                alliance = DiegoPathing.Alliance.values()[(alliance.ordinal() + 1)%2];
            }
            telemetry.addData("ALLIANCE", alliance);

            // update drive

            drive.updateOdometry();
            Pose pose = drive.getPose();
            telemetry.addData("Pose", "X %.1f  Y %.1f  H %.2f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            boolean lb1 = gamepad1.leftBumperWasPressed();
            boolean lb2 = gamepad2.leftBumperWasPressed();
            if (lb1 || lb2) robotCentric = !robotCentric;

            boolean b1 = gamepad1.bWasPressed();
            boolean b2 = gamepad2.bWasPressed();

            if (b1 || b2) {
                pose = new Pose(0,0,0);
               drive.setPose(pose);
               positionToHold = pose;
            }

            boolean dpl1 = gamepad1.dpadLeftWasPressed();
            boolean dpl2 = gamepad2.dpadLeftWasPressed();

            if (dpl1 || dpl2) {
                shootingPose = drive.getPose();
            }
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

             boolean dpd1 = gamepad1.dpadDownWasPressed();
             boolean dpd2 = gamepad2.dpadDownWasPressed();

            if (dpd1 || dpd2){
                slowMode = !slowMode;
                speedScaler = slowMode? 0.35 : 1.0;
            }

            if (gamepad2.rightBumperWasPressed()){
                cubicMode = !cubicMode;
            }

            boolean back1WasPressed = gamepad1.backWasPressed();
            boolean back2WasPressed = gamepad2.backWasPressed();

            if (autoDrive == null && back1WasPressed) {
                autoDrive = new DriveToTarget(shootingPose);
                holdingPosition = false;
            } else if (autoDrive == null && back2WasPressed){
                autoDrive = new TurnToTarget();
                holdingPosition = false;
            } else if (autoDrive != null){
                if (autoDrive.update() || (autoDrive instanceof DriveToTarget && !gamepad1.back
                || autoDrive instanceof TurnToTarget && !gamepad2.back)){
                    autoDrive = null;
                    drive.setDrivePower(0,0,0);
                }
            }

            if (gamepad2.xWasPressed()) enableHold = !enableHold;
            telemetry.addData("holdingPose",holdingPosition);
            telemetry.addData("autodrive", autoDrive !=null);

            if (autoDrive == null){
                double px = -gamepad1.left_stick_y;
                double py = -gamepad1.left_stick_x ;
                double pa = gamepad1.left_trigger-gamepad1.right_trigger;

                if (cubicMode){
                    px = px * px * px;
                    py = py * py * py;
                    pa = pa * pa * pa;
                }

                px = px * speedScaler;
                py = py * speedScaler;
                pa = pa * speedScaler;

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

            telemetry.addData("Drive Current", drive.getDriveCurrent());

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
                shooter.setTargetSpeed(shootFast? fastShootSpeed : slowShootSpeed);
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
            boolean y1 = gamepad1.yWasPressed();
            boolean y2 = gamepad2.yWasPressed();
            if (y1 || y2) jeff.moveNext();

            telemetry.addData("jeff Index", jeff.getIndex());
            telemetry.addData("robot centric", robotCentric);
            telemetry.addData("slowmode", slowMode);
            telemetry.addData("cubicMode", cubicMode);
            telemetry.update();

        }
    }

    interface AutoDrive{
        boolean update();
    }

    class DriveToTarget implements AutoDrive{
        Pose startPose;
        Pose targetPose;

        public DriveToTarget(Pose targetPose) {
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

    class TurnToTarget implements AutoDrive {
        Pose startPose;
        Pose targetPose = null;
        int tagsFound = 0;
        public TurnToTarget(){
            startPose = drive.getPose();
            targetPose = startPose;
            List<AprilTagDetection> tags = apriltag.getTags();
            OpenGLMatrix tagToRobot = getTagToRobot(tags);
            if (tagToRobot == null) return;
            tagsFound = tagsFound + 1;
            double bearing = Math.atan2(tagToRobot.get(1,3), tagToRobot.get(0,3));
            targetPose = new Pose(startPose.getX(), startPose.getY(),
                    AngleUnit.normalizeRadians(startPose.getHeading() + bearing));
        }

        public boolean update(){
            drive.updateOdometry();
            Pose pose = drive.getPose();
            List<AprilTagDetection> tags = apriltag.getFreshTags();
            OpenGLMatrix tagToRobot = getTagToRobot(tags);
            if (tagToRobot != null) {
                double dist = Math.hypot(tagToRobot.get(0,3), tagToRobot.get(1,3));
                double bearing = Math.atan2(tagToRobot.get(1,3), tagToRobot.get(0,3));
                if (dist > 104){
                    bearing += alliance == DiegoPathing.Alliance.RED ? -Math.toRadians(0) : Math.toRadians(3);
                }
                targetPose = new Pose(startPose.getX(), startPose.getY(),
                        AngleUnit.normalizeRadians(pose.getHeading() + bearing + Math.PI));
                telemetry.addData("dist", dist);
            }
            diego.holdPose(targetPose, 2.0, 8.0);
            telemetry.addData("AUTO TURN TAGS FOUND", tagsFound);
            return false;
        }

        private OpenGLMatrix getTagToRobot(List<AprilTagDetection> tags){
            if (tags == null || tags.isEmpty()) return null;
            AprilTagDetection tag = null;
            for(AprilTagDetection t : tags){
                if (t.id == 20 && alliance == DiegoPathing.Alliance.BLUE
                        || t.id == 24 && alliance == DiegoPathing.Alliance.RED){
                    tag = t;
                }
            }
            if (tag == null) return null;
            return apriltag.aprilTagPose(tag);
        }

    }

}
