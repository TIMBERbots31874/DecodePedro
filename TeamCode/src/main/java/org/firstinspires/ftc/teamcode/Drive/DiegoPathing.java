package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DiegoPathing {

    private Motion motion;
    private LinearOpMode opMode;

    public final double HOLD_POSE_COEFF = 2.0;
    public final double HOLD_HEADING_COEFF = 2.0;
    public final double CORRECT_HEADING_COEFF = 2.0;

    public enum Alliance {BLUE, RED}

    public DiegoPathing(Motion motion, LinearOpMode opMode){
        this.motion = motion;
        this.opMode = opMode;
    }

    public void waitAsync(double millis){
        ElapsedTime et = new ElapsedTime();
        while(et.milliseconds()< millis && opMode.opModeIsActive()){
            motion.updateOdometry();
        }
    }

    public void waitAsync(double millis, Runnable runnable){
        ElapsedTime et = new ElapsedTime();
        while(et.milliseconds() < millis && opMode.opModeIsActive()){
            motion.updateOdometry();
            runnable.run();
        }
    }

    public void holdPoseAsync(double millis, Pose targetPose, Runnable runnable){
        ElapsedTime et = new ElapsedTime();
        while (et.milliseconds() < millis && opMode.opModeIsActive()){
            motion.updateOdometry();
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
            double headingSpeed = 2.0 * headingError;

            motion.setDriveSpeed(velRobot.get(0), velRobot.get(1), headingSpeed);
        }
        motion.setDrivePower(0,0,0);
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
            headingSpeed = CORRECT_HEADING_COEFF * headingError;
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



    public static VectorF fieldToRobot(VectorF vField, double heading){
        float sin = (float)Math.sin(heading);
        float cos = (float)Math.cos(heading);
        return new VectorF(vField.get(0)*cos + vField.get(1)*sin,
                -vField.get(0)*sin + vField.get(1)*cos);
    }

}
