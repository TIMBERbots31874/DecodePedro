package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Apriltag {
    AprilTagProcessor processor;
    VisionPortal portal;

    OpenGLMatrix cameraToRobot = OpenGLMatrix.translation(8, 0, 0)
            .rotated(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, -90, -90, 0);


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

    public List<AprilTagDetection> getFreshTags() { return processor.getFreshDetections(); }

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

    public OpenGLMatrix aprilTagPose(AprilTagDetection tag){
        MatrixF R = tag.rawPose.R;
        OpenGLMatrix tagToCamera = new OpenGLMatrix(new float[]{
                R.get(0, 0), R.get(1, 0), R.get(2, 0), 0,
                R.get(0, 1), R.get(1, 1), R.get(2, 1), 0,
                R.get(0, 2), R.get(1, 2), R.get(2, 2), 0,
                (float) tag.rawPose.x, (float) tag.rawPose.y, (float) tag.rawPose.z, 1
        });
        return cameraToRobot.multiplied(tagToCamera);
    }

}
