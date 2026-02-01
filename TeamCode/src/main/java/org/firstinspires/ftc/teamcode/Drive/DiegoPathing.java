package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pathutil.CubicSpline2D;

public class DiegoPathing {

    private Motion motion;
    private LinearOpMode opMode;

    public final double HOLD_POSE_COEFF = 2.0;
    public final double HOLD_HEADING_COEFF = 2.0;
    public final double CORRECT_HEADING_COEFF = 2.0;
    public final double CORRECT_POSE_COEFF = 6.0;
    public final double CURVATURE_COEFF = 0.1;

    public final double STD_TRANSLATION_TOLERANCE = 1;
    public final double STD_HEADING_TOLERANCE = Math.toRadians(1);

    public final double STD_TMAX = 0.98;

    public enum Alliance {BLUE, RED}

    public enum HeadingMode {CONSTANT, LINEAR, TANGENT}

    public static final boolean LOGGING = false;

    public DiegoPathing(Motion motion, LinearOpMode opMode){
        this.motion = motion;
        this.opMode = opMode;
    }

    public void waitAsync(double millis){
        ElapsedTime et = new ElapsedTime();
        while(et.milliseconds()< millis && opMode.opModeIsActive()){
            motion.updateOdometry();
            if (LOGGING) {
                Pose pose = motion.getPose();
                reportPose(pose);
            }
        }
    }

    public void waitAsync(double millis, Runnable runnable){
        ElapsedTime et = new ElapsedTime();
        while(et.milliseconds() < millis && opMode.opModeIsActive()){
            motion.updateOdometry();
            if (LOGGING) {
                Pose pose = motion.getPose();
                reportPose(pose);
            }
            runnable.run();
        }
    }

    public void holdPoseAsync(double millis, Pose targetPose, Runnable runnable){
        ElapsedTime et = new ElapsedTime();
        while (et.milliseconds() < millis && opMode.opModeIsActive()){
            motion.updateOdometry();
            if (LOGGING) {
                Pose pose = motion.getPose();
                reportPose(pose);
            }
            if (runnable != null) runnable.run();
            holdPose(targetPose);
        }
        motion.setDrivePower(0, 0, 0);
    }

    public void holdPoseAsync(double millis, Pose targetPose){
        holdPoseAsync(millis, targetPose, null);
    }

    public void holdPose(Pose targetPose){
        Pose pose = motion.getPose();
        VectorF dPoseRobot = fieldToRobot(
                new VectorF((float)(targetPose.getX()-pose.getX()), (float)(targetPose.getY()-pose.getY())),
                pose.getHeading());
        double dHeading = AngleUnit.normalizeRadians(targetPose.getHeading() - pose.getHeading());
        if (dPoseRobot.magnitude() < 1 && Math.abs(Math.toDegrees(dHeading)) < 1 ){
            motion.setDrivePower(0,0,0);
            return;
        }
        motion.setDriveSpeed(HOLD_POSE_COEFF*dPoseRobot.get(0),
                HOLD_POSE_COEFF*dPoseRobot.get(1), HOLD_HEADING_COEFF*dHeading);
    }

    public void driveTo(Pose targetPose, MotionProfile mProf, double tolerance){
        driveTo(targetPose, mProf, tolerance, null);
    }


    public void driveTo(Pose targetPose, MotionProfile mProf, double tolerance, Runnable runnable){
        motion.updateOdometry();
        Pose startPose = motion.getPose();
        VectorF startVec = new VectorF((float)startPose.getX(), (float)startPose.getY());
        double targetHeading = targetPose.getHeading();
        VectorF targetVec = new VectorF((float)targetPose.getX(), (float)targetPose.getY());

        while (opMode.opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            if (LOGGING) reportPose(pose);
            VectorF poseVec = new VectorF((float)pose.getX(), (float)pose.getY());
            VectorF d1 = poseVec.subtracted(startVec);
            VectorF d2 = targetVec.subtracted(poseVec);
            float d1Mag = d1.magnitude();
            float d2Mag = d2.magnitude();

            if (d2Mag < tolerance) break;
            if (runnable != null) runnable.run();

            float speed1 = (float)Math.sqrt(mProf.v0*mProf.v0 + 2.0*mProf.accel*d1Mag);
            float speed2 = (float)Math.sqrt(mProf.v0*mProf.v0 + 2.0*mProf.accel*d2Mag);
            float speed = Math.min(mProf.vMax, Math.min(speed1, speed2));

            VectorF vel = d2.multiplied(speed/d2Mag);
            VectorF velRobot = fieldToRobot(vel, pose.getHeading());

            double headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
            double headingSpeed = CORRECT_HEADING_COEFF * headingError;

            motion.setDriveSpeed(velRobot.get(0), velRobot.get(1), headingSpeed);
        }
        motion.setDrivePower(0,0,0);
    }

    public void driveTo(Pose targetPose, MotionProfile mProf, double tolerance,
                        double headingToleranceDegrees, Runnable runnable){
        Pose startPose = motion.getPose();

        while (opMode.opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            if (LOGGING) reportPose(pose);

            double error = Math.hypot(pose.getY()- targetPose.getY(), pose.getX() - targetPose.getX());
            double turnError = Math.toDegrees(
                    AngleUnit.normalizeRadians(targetPose.getHeading() - pose.getHeading()));

            if (error < tolerance && Math.abs(turnError) < headingToleranceDegrees) break;

            if (runnable != null) runnable.run();

            driveToward(startPose, targetPose, mProf, tolerance);
        }

        motion.setDrivePower(0, 0, 0);
    }


    public void driveToward(Pose startPose, Pose targetPose, MotionProfile mProf, double tolerance) {
        VectorF startVec = new VectorF((float) startPose.getX(), (float) startPose.getY());
        VectorF targetVec = new VectorF((float) targetPose.getX(), (float) targetPose.getY());
        Pose pose = motion.getPose();
        VectorF poseVec = new VectorF((float) pose.getX(), (float) pose.getY());
        VectorF d1 = poseVec.subtracted(startVec);
        VectorF d2 = targetVec.subtracted(poseVec);
        float d1Mag = d1.magnitude();
        float d2Mag = d2.magnitude();
        float speed1 = (float) Math.sqrt(mProf.v0 * mProf.v0 + 2.0 * mProf.accel * d1Mag);
        float speed2 = (float) Math.sqrt(mProf.v0 * mProf.v0 + 2.0 * mProf.accel * d2Mag);
        float speed = Math.min(mProf.vMax, Math.min(speed1, speed2));
        VectorF vel = d2Mag > tolerance? d2.multiplied(speed / d2Mag) : new VectorF(0,0);
        VectorF velRobot = fieldToRobot(vel, pose.getHeading());
        float dTotal = targetVec.subtracted(startVec).magnitude();

        double headingError, headingSpeed, targetHeading;
        if (d2Mag<=tolerance || dTotal <= tolerance){
            targetHeading = targetPose.getHeading();
            headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
            headingSpeed = 2.0 * CORRECT_HEADING_COEFF * headingError;
            headingSpeed = Range.clip(headingSpeed, -1.5f, 1.5f);
        } else {
            double totalHeadingChange = AngleUnit.normalizeRadians(targetPose.getHeading()-startPose.getHeading());
            targetHeading = AngleUnit.normalizeRadians(
                    startPose.getHeading() + totalHeadingChange * (dTotal-d2Mag) / dTotal);
            headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
            headingSpeed = totalHeadingChange * vel.magnitude() / dTotal + CORRECT_HEADING_COEFF * headingError;
            headingSpeed = Range.clip(headingSpeed, -1.5f, 1.5f);
        }

        motion.setDriveSpeed(velRobot.get(0), velRobot.get(1), headingSpeed);
    }

    public void turnTo(double targetHeadingDegrees, double maxDegreesPerSec, double pTurn, double toleranceDegrees){
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        double vaMax = Math.toRadians(maxDegreesPerSec);
        double tolerance = Math.toRadians(toleranceDegrees);
        while (opMode.opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            if (LOGGING) reportPose(pose);
            Pose vel = motion.getVelocity();
            double headingOffset = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
            if (Math.abs(headingOffset) < tolerance
                && Math.abs(vel.getHeading()) < 10*tolerance){
                break;
            }
            double va = pTurn * headingOffset;
            va = Range.clip(va, -vaMax, vaMax);
            if (Math.abs(va) < 5*tolerance) va = Math.signum(va) * 5.0 * tolerance;
            motion.setDriveSpeed(0, 0, va);
        }
        motion.setDrivePower(0,0,0);
    }


    public void turnTo(double targetHeadingDegrees, double maxDegreesPerSec, double pTurn,
                       double toleranceDegrees, Runnable runnable){
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        double vaMax = Math.toRadians(maxDegreesPerSec);
        double tolerance = Math.toRadians(toleranceDegrees);
        while (opMode.opModeIsActive()){
            motion.updateOdometry();
            Pose pose = motion.getPose();
            if (LOGGING) reportPose(pose);
            Pose vel = motion.getVelocity();
            double headingOffset = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
            if (Math.abs(headingOffset) < tolerance
                    && Math.abs(vel.getHeading()) < 10*tolerance){
                break;
            }
            runnable.run();
            double va = pTurn * headingOffset;
            va = Range.clip(va, -vaMax, vaMax);
            if (Math.abs(va) < 5*tolerance) va = Math.signum(va) * 5.0 * tolerance;
            motion.setDriveSpeed(0, 0, va);
        }
        motion.setDrivePower(0,0,0);
    }

    public void driveSpline(CubicSpline2D spline, MotionProfile mProf, HeadingMode headingMode, boolean reversed, Runnable runnable) {
        double dist = spline.getTotalPathLength();
        Pose startPose = spline.getStartPose();
        double startHeading = startPose.getHeading();
        Pose endPose = spline.getEndPose();
        double t = 0;

        if (headingMode == HeadingMode.CONSTANT) {
            endPose = new Pose(endPose.getX(), endPose.getY(), startHeading);
        } else if (headingMode == HeadingMode.TANGENT) {
            VectorF d1End = spline.d1(1);
            endPose = reversed? new Pose(endPose.getX(), endPose.getY(), AngleUnit.normalizeRadians(Math.atan2(d1End.get(1), d1End.get(0))+Math.PI))
                    : new Pose(endPose.getX(), endPose.getY(), Math.atan2(d1End.get(1), d1End.get(0)));
        }

        while (opMode.opModeIsActive()) {
            motion.updateOdometry();
            Pose pose = motion.getPose();
            if (LOGGING) reportPose(pose);
            VectorF pos = new VectorF((float) pose.getX(), (float) pose.getY());
            if (Math.hypot(endPose.getX()-pose.getX(), endPose.getY()-pose.getY()) < STD_TRANSLATION_TOLERANCE
                    && Math.abs(AngleUnit.normalizeRadians(pose.getHeading() - endPose.getHeading())) < STD_HEADING_TOLERANCE) {
                break;
            }

            if (runnable != null) runnable.run();

            t = spline.getClosestT(pose.getX(), pose.getY(), t);

            if (t < STD_TMAX) {
                double dist1 = spline.getPathLength(t);
                double dist2 = dist - dist1;

                float speed1 = (float) Math.sqrt(mProf.v0 * mProf.v0 + 2.0 * mProf.accel * dist1);
                float speed2 = (float) Math.sqrt(mProf.v0 * mProf.v0 + 2.0 * mProf.accel * dist2);
                float speed = Math.min(mProf.vMax, Math.min(speed1, speed2));


                Pose currentVelocity = motion.getVelocity();
                double currentSpeed = Math.hypot(currentVelocity.getX(), currentVelocity.getY());
                VectorF closestPos = spline.p(t);
                VectorF d1 = spline.d1(t);
                VectorF splineDir = d1.multiplied(1.0f / d1.magnitude());
                VectorF splineNorm = new VectorF(-splineDir.get(1), splineDir.get(0));
                double curvature = spline.getCurvature(t);
                VectorF err = pos.subtracted(closestPos);
                VectorF normErr = splineNorm.multiplied(splineDir.get(0)*err.get(1)-splineDir.get(1)*err.get(0));
                VectorF curvatureCorrection = splineNorm.multiplied((float)(curvature*currentSpeed*CURVATURE_COEFF));
                VectorF nominalDriveDir = splineDir.added(curvatureCorrection);
                nominalDriveDir = nominalDriveDir.multiplied(1.0f/nominalDriveDir.magnitude());
                VectorF vel = nominalDriveDir.multiplied(speed).subtracted(normErr.multiplied((float)CORRECT_POSE_COEFF));
                VectorF velRobot = fieldToRobot(vel, pose.getHeading());

                double targetHeading;
                double nominalHeadingVel;
                if (headingMode == HeadingMode.CONSTANT) {
                    targetHeading = startHeading;
                    nominalHeadingVel = 0;
                } else if (headingMode == HeadingMode.LINEAR) {
                    double headingChange = AngleUnit.normalizeRadians(endPose.getHeading() - startHeading);
                    targetHeading = AngleUnit.normalizeRadians(startHeading + headingChange * dist1 / dist);
                    nominalHeadingVel = speed * headingChange / dist;
                } else {
                    targetHeading = Math.atan2(d1.get(1), d1.get(0));
                    if (reversed)
                        targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
                    nominalHeadingVel = speed * curvature;
                }

                double headingErr = AngleUnit.normalizeRadians(targetHeading - pose.getHeading());
                double headingVel = nominalHeadingVel + CORRECT_HEADING_COEFF * headingErr;

                motion.setDriveSpeed(velRobot.get(0), velRobot.get(1), headingVel);
            } else {
                holdPose(endPose);
            }
        }

        motion.setDrivePower(0, 0, 0);
    }



    public static VectorF fieldToRobot(VectorF vField, double heading){
        float sin = (float)Math.sin(heading);
        float cos = (float)Math.cos(heading);
        return new VectorF(vField.get(0)*cos + vField.get(1)*sin,
                -vField.get(0)*sin + vField.get(1)*cos);
    }

    public void reportPose(Pose pose){
        opMode.telemetry.addData("Pose", "X: %.2f  Y: %.2f  H: %.2f", pose.getX(),
                pose.getY(), Math.toDegrees(pose.getHeading()));
        opMode.telemetry.update();
    }



}
