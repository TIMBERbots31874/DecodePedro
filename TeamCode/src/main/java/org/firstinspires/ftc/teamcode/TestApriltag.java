package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Apriltag;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TestApriltag extends LinearOpMode {

    Apriltag aprilTag;

    public void runOpMode(){

        aprilTag = new Apriltag(hardwareMap);

        boolean streaming = false;
        ElapsedTime et = new ElapsedTime();
        while (opModeInInit()) {
            VisionPortal.CameraState state = aprilTag.getCameraState();
            if (state == VisionPortal.CameraState.STREAMING) streaming = true;
            telemetry.addData("camera state", state);
            telemetry.addData("streaming", streaming);
            telemetry.update();
        }


        while(opModeIsActive()){
            telemetry.addData("streaming",streaming);
            if (streaming){

                int id = aprilTag.getObeliskID();
                telemetry.addData("id",id);
            }
            telemetry.update();
        }
    }
}
