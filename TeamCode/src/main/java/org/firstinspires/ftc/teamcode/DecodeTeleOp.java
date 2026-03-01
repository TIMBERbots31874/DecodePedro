package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.util.AnalogToggle;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/*
 * Gamepad Mappings
 *
 * Gamepad1:
 *      left_stick: drive (translation)
 *      right_stick_x: lift "up and down"
 *      dpad down: slowmode toggle;     up: nothing  ;    left: Set Shooting pose    right: Shooter Speed
 *      A: intakeState REV     B: setPose;     X: Kicker;      Y: Spinner
 *      leftBumper: robotCentric (toggle);   rightBumper: intakeState FWD
 *      back: autoDrive (press and hold)
 *      trigger left: turning;        right: turning;
 *
 * gamepad2
 *
 *      a: toggle alliance  b: reset pose   x: toggle hold pose  y: spinner
 *      dpad: left: set shooting pose     dpad down: toggle slow mode    dpad Up:Lift and servo
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

    double px = 0;
    double py = 0;
    double pa = 0;
    AnalogToggle pxToggle = new AnalogToggle(()-> (float)Math.abs(px), 0.6,0.2);
    AnalogToggle pyToggle = new AnalogToggle(()->(float)Math.abs(py), 0.6, 0.2);
    AnalogToggle paToggle = new AnalogToggle(()->(float)Math.abs(pa), 0.6, 0.2);
    boolean robotCentric = true;
    boolean slowMode = false;
    boolean cubicMode = false;
    double speedScaler = 1;


    Automation autoDrive = null;
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
    boolean fineAdjust = false;


    Automation autoShoot = null;
    enum AutoShootState {ENGAGE, RELEASE, ADVANCE}
    interface Predicate{boolean test();}




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

            if (gamepad2.dpadRightWasPressed()){
                fineAdjust = !fineAdjust;
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
            telemetry.addData("fineAdjust", fineAdjust);

            px = -gamepad1.left_stick_y;
            py = -gamepad1.left_stick_x ;
            pa = gamepad1.left_trigger-gamepad1.right_trigger;

            boolean pxToggled = pxToggle.update();
            boolean pyToggled = pyToggle.update();
            boolean paToggled = paToggle.update();

            if (autoDrive == null){

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
                    if ((px != 0 || py!=0 || pa!=0 || !enableHold) && !fineAdjust) holdingPosition = false;
                } else {
                    if (px==0 && py==0 && pa==0 && enableHold && stopped) holdingPosition=true;
                    positionToHold=drive.getPose();
                }


                if (holdingPosition){
                    if (fineAdjust && !robotCentric){
                        double deltaXR = 0;
                        double deltaYR = 0;
                        double deltaH = 0;
                        if(pxToggled){
                            deltaXR = 0.25 * Math.signum(px) * (alliance == DiegoPathing.Alliance.RED? 1:-1);
                        }
                        if(pyToggled){
                            deltaYR = 0.25 * Math.signum(py) * (alliance == DiegoPathing.Alliance.RED? 1:-1);
                        }
                        if (paToggled){
                            deltaH = 0.025 * Math.signum(pa);
                        }
                        double sin = Math.sin(pose.getHeading());
                        double cos = Math.cos(pose.getHeading());
                        double deltaX = deltaXR * cos - deltaYR * sin;
                        double deltaY = deltaXR * sin + deltaYR * cos;
                        positionToHold = new Pose(positionToHold.getX() + deltaX,
                                positionToHold.getY() + deltaY, positionToHold.getHeading() + deltaH);
                        diego.holdPose(positionToHold, 16, 16);
                    } else {
                        diego.holdPose(positionToHold);
                    }
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
            if (gamepad2.dpadUpWasPressed()){
                liftLocked = !liftLocked;
                if (liftLocked) lift.lockLift();
                else lift.releaseLift();
            }

            if (!gamepad2.dpad_up || Math.abs(gamepad2.right_stick_y) < 0.5){
                if (operatingLift) lift.holdPosition();
                else lift.setPower(0);
            } else if (gamepad2.right_stick_y < -0.5) {
                operatingLift = true;
                lift.setPower(1);
            } else if (gamepad2.right_stick_y > 0.5) {
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
                    if (rightBump1pressed || operatingLift) {
                        intakeState = Intake.State.STOPPED;
                        jeff.setOffSet(false);
                    }
                    else if (a1Pressed) {
                        intakeState = Intake.State.REVERSE;
                        jeff.setOffSet(true);
                    }
                    break;
                case REVERSE:
                    if (a1Pressed || operatingLift) {
                        intakeState = Intake.State.STOPPED;
                        jeff.setOffSet(false);
                    }
                    else if (rightBump1pressed) {
                        intakeState = Intake.State.FORWARD;
                        jeff.setOffSet(false);
                    }
                    break;
                case STOPPED:
                    if (operatingLift) {
                        intakeState = Intake.State.STOPPED;
                        jeff.setOffSet(false);
                    }
                    else if (rightBump1pressed) {
                        intakeState = Intake.State.FORWARD;
                        jeff.setOffSet(false);
                    }
                    else if (a1Pressed) {
                        intakeState = Intake.State.REVERSE;
                        jeff.setOffSet(true);
                    }
            }
            intake.setState(intakeState);

            // update shooter
            if (gamepad1.dpadRightWasPressed()){
                shootFast = !shootFast;
                shooter.setTargetSpeed(shootFast? fastShootSpeed : slowShootSpeed);
            }

            if (operatingLift) shooter.setSpeed(0);
            else shooter.update();


            if (autoShoot == null && gamepad2.right_trigger>0.4){
                autoShoot = new AutoShoot(()->gamepad2.right_trigger>0.4);
            } else if(autoShoot != null){
                if (autoShoot.update()){
                    autoShoot = null;
                }
            }

            if (autoShoot == null) {
                if (gamepad1.x && !jeff.getOffSet()) shooter.engageKicker();
                else shooter.releaseKicker();
            }

            telemetry.addData("shooter speeds", "R %.2f  L %.2f",
                    shooter.getRightSpeed(), shooter.getLeftSpeed());
            telemetry.addData("shooter powers", "R %.3f  L %.3f",
                    shooter.getRightPower(), shooter.getLeftPower());


            // update turntable

            if (autoShoot == null) {
                boolean y1 = gamepad1.yWasPressed();
                boolean y2 = gamepad2.yWasPressed();
                if (y1 || y2) jeff.moveNext();
            }

            telemetry.addData("jeff Index", jeff.getIndex());
            telemetry.addData("robot centric", robotCentric);
            telemetry.addData("slowmode", slowMode);
            telemetry.addData("cubicMode", cubicMode);
            telemetry.update();

        }
    }

    interface Automation {
        boolean update();
    }

    class DriveToTarget implements Automation {
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
            diego.driveToward(startPose, targetPose, new MotionProfile(12,48,36), 1); //v06
            return false;
        }
    }

    class TurnToTarget implements Automation {
        Pose startPose;
        Pose targetPose = null;
        int tagsFound = 0;
        public TurnToTarget(){
            startPose = drive.getPose();
            targetPose = startPose;
            List<AprilTagDetection> tags = apriltag.getTags();
            OpenGLMatrix cornerToRobot = getCornerToRobot(tags);
            if (cornerToRobot == null) return;
            tagsFound = tagsFound + 1;
            double bearing = Math.atan2(cornerToRobot.get(1,3), cornerToRobot.get(0,3));
            targetPose = new Pose(startPose.getX(), startPose.getY(),
                    AngleUnit.normalizeRadians(startPose.getHeading() + bearing + Math.PI));
        }

        public boolean update(){
            drive.updateOdometry();
            Pose pose = drive.getPose();
            List<AprilTagDetection> tags = apriltag.getFreshTags();
            OpenGLMatrix cornerToRobot = getCornerToRobot(tags);
            if (cornerToRobot != null) {
                tagsFound += 1;
                double bearing = Math.atan2(cornerToRobot.get(1,3), cornerToRobot.get(0,3));
                targetPose = new Pose(startPose.getX(), startPose.getY(),
                        AngleUnit.normalizeRadians(pose.getHeading() + bearing + Math.PI));
            }
            diego.holdPose(targetPose, 2.0, 8.0);
            telemetry.addData("AUTO TURN TAGS FOUND", tagsFound);
            return false;
        }





    }

    private OpenGLMatrix getCornerToRobot(List<AprilTagDetection> tags){
        if (tags == null || tags.isEmpty()) return null;
        AprilTagDetection tag = null;
        for(AprilTagDetection t : tags){
            if (t.id == 20 && alliance == DiegoPathing.Alliance.BLUE
                    || t.id == 24 && alliance == DiegoPathing.Alliance.RED){
                tag = t;
            }
        }
        if (tag == null) return null;
        return apriltag.cornerToRobot(tag, alliance);
    }

    public class AutoShoot implements Automation{
        private AutoShootState state = AutoShootState.ENGAGE;
        private ElapsedTime et = new ElapsedTime();
        private Predicate predicate;

        public AutoShoot(Predicate p){
            predicate = p;
            et.reset();
            shooter.engageKicker();
        }

        @Override
        public boolean update() {
            switch(state){
                case ENGAGE:
                    if (et.milliseconds()>350){
                        state = AutoShootState.RELEASE;
                        et.reset();
                        shooter.releaseKicker();
                    }
                    break;
                case RELEASE:
                    if (et.milliseconds()>450){
                        state = AutoShootState.ADVANCE;
                        et.reset();
                        jeff.moveNext();
                    }
                    break;
                case ADVANCE:
                    if (et.milliseconds() > 500) {
                        if (predicate.test()) {
                            state = AutoShootState.ENGAGE;
                            et.reset();
                            shooter.engageKicker();
                        } else {
                            return true;
                        }
                    }
            }
            return false;
        }
    }


}
