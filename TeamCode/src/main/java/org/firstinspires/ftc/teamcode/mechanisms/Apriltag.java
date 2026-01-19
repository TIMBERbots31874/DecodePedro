package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Apriltag {
    AprilTagProcessor processor;
    VisionPortal portal;



    public Apriltag(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .build();

        processor.setDecimation(3);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();
    }

    public List<AprilTagDetection> getTags(){
        return processor.getDetections();
    }

    public void setDecimation(float decimation){
        processor.setDecimation(decimation);
    }

    public int getObeliskID(){
        List<AprilTagDetection> tags = getTags();
        if (tags == null || tags.isEmpty()) return 0;
        for (AprilTagDetection tag: tags){
           if (tag.id >= 21 && tag.id <= 23 ) return tag.id;
        }
        return 0;
    }

    public VisionPortal.CameraState getCameraState(){
        return portal.getCameraState();
    }

    public boolean streaming(){
        return getCameraState() == VisionPortal.CameraState.STREAMING;
    }

}
