package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestGampad extends LinearOpMode {
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("righttrigger", gamepad1.right_trigger);
            telemetry.addData("lefttrigger", gamepad1.left_trigger);
            telemetry.addData("leftstickx", gamepad1.left_stick_x);
            telemetry.addData("leftsticky", gamepad1.left_stick_y);
            telemetry.addData("rightstickx", gamepad1.right_stick_x);
            telemetry.addData("rightsticky", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
