package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drive.Motion;

@TeleOp
public class TestServo extends LinearOpMode {

    Servo servo;

    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0.5); //*Calls the standard set position on the servo

        telemetry.addLine("Ready to test servo");
        telemetry.update();

        waitForStart();

        double pos = 0.5;

        while (opModeIsActive()){
            if(gamepad1.dpad_right) pos +=0.0001;
            else if(gamepad1.dpad_left) pos -=0.0001;

            pos = Range.clip(pos, 0, 1);

            servo.setPosition(pos);

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}