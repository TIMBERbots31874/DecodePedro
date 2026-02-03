package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.Motion;

@TeleOp
public class TestIMUYaw extends LinearOpMode {

    Motion drive;

    public void runOpMode(){

        drive = new Motion(this);

        waitForStart();

        drive.setPose(new Pose(0,0,0));

        double poseAdjustment = 0;

        Pose pose = new Pose(0,0,0);

        while (opModeIsActive()){
            drive.updateOdometry();
            Pose newPose = drive.getPose();

            if (newPose.getHeading() - pose.getHeading() > Math.PI) poseAdjustment -= 2.0 * Math.PI;
            else if (newPose.getHeading() - pose.getHeading() < -Math.PI) poseAdjustment += 2.0 * Math.PI;

            pose = newPose;
            double cumulativeHeading = pose.getHeading() + poseAdjustment;

            telemetry.addData("Pose", "X: %.2f  Y: %.2f  H: %.2f", pose.getX(),
                    pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Cumulative Heading", Math.toDegrees(cumulativeHeading));
            telemetry.update();
            drive.setDrivePower(0, 0, -0.2 * gamepad1.right_stick_x);
        }

    }

}
