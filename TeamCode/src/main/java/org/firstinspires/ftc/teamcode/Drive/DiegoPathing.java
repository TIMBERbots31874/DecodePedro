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

    public void driveTo(Pose targetPose, MotionProfile mProf, double tolerance){
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

    public static VectorF fieldToRobot(VectorF vField, double heading){
        float sin = (float)Math.sin(heading);
        float cos = (float)Math.cos(heading);
        return new VectorF(vField.get(0)*cos + vField.get(1)*sin,
                -vField.get(0)*sin + vField.get(1)*cos);
    }

}
