package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class Drive extends LinearOpMode {

    public Motion drive;

    public void runOpMode(){
        drive = new Motion(this); //This generates a new instance of the motion class); this is a constructor method

        // Calling a method
      drive.setPose(new Pose(0,0,0));

        waitForStart();

        while(opModeIsActive()){
            double px = - gamepad1.left_stick_y;
            double py = - gamepad1.left_stick_x;
            double pa = - gamepad1.right_stick_x;
            drive.setDrivePower(px, py, pa);
            drive.updateOdometry();
            Pose pose = drive.getPose();
            telemetry.addData("Pose", "X: %.1f  Y:  %.1f  H:  %.1f",
                    pose.getX(), pose.getY(),
                    Math.toDegrees(pose.getHeading()));
            int xTicks = drive.odo.getEncoderX();
            int yTicks = drive.odo.getEncoderY();
            telemetry.addData(" Ticks", "X Ticks:  %d   Y Ticks:   %d", xTicks, yTicks);
            telemetry.update();



        }






    }



}


