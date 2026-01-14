package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestV3 extends LinearOpMode {

    RevColorSensorV3 color;

    public void runOpMode(){

        color = hardwareMap.get(RevColorSensorV3.class, "color");
        waitForStart();

        while (opModeIsActive()) {
            int red = color.red() * 255 / 360;
            int green = color.green() * 255 / 360;
            int blue = color.blue() * 255 / 360;
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);
            telemetry.addData("rgb", "%d  %d  %d", red, green, blue);
            telemetry.addData("hsv", "%.3f  %.3f  %.3f", hsv[0], hsv[1], hsv[2]);
            telemetry.update();
        }
    }
}