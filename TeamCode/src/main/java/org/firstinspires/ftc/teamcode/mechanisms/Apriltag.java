package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Apriltag {
    AprilTagProcessor processor;
    VisionPortal portal;

    enum Obelisk{O21, O22, O23, NONE}

    public Apriltag(HardwareMap hardwareMap){
        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,
                "Webcam 1"));
    }

    public List<AprilTagDetection> getTags(){
        return processor.getDetections();
    }

    public Obelisk getObeliskID(){
        List<AprilTagDetection> tags = getTags();
        if (tags == null || tags.isEmpty()) return Obelisk.NONE;
        return Obelisk.NONE;
    }

}
