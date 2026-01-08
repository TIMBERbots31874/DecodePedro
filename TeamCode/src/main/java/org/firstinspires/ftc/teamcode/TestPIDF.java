package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Drive.DiegoPathing;
import org.firstinspires.ftc.teamcode.Drive.Motion;
import org.firstinspires.ftc.teamcode.Drive.MotionProfile;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

import java.util.HashMap;
import java.util.function.BiConsumer;

@Configurable
@TeleOp
public class TestPIDF extends LinearOpMode {

    public static double NEXT_P = 0;
    public static double NEXT_D = 0;
    public static double NEXT_F = 0;
    public static double DIST = 60;
    public double speed = 0;
    public double maxSpeed = 0;

    public double tempMaxSpeed = 0;

    MotionProfile mProf = new MotionProfile(6, 36, 30);

    Motion drive;
    DiegoPathing diego;
    Lift lift;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public void runOpMode(){
        drive = new Motion(this);
        diego = new DiegoPathing(drive, this);
        lift = new Lift(hardwareMap);
        lift.lockLift();

        PIDFCoefficients pidf_Initial = drive.getPIDFCoefficients();
        NEXT_P = pidf_Initial.p;
        NEXT_D = pidf_Initial.d;
        NEXT_F = pidf_Initial.f;

        HashMap<String,PIDFCoefficients> pidfMap = drive.getAllPIDFCoefficients();

        telemetry.addData("Starting PIDFs","");
        for (HashMap.Entry<String,PIDFCoefficients> entry: pidfMap.entrySet()){
            telemetry.addData(entry.getKey(), "P %.4f  I %.4f  D %.4f  F %.4f",
                    entry.getValue().p, entry.getValue().i, entry.getValue().d, entry.getValue().f);
        }
        telemetry.update();

        waitForStart();

        Pose pose0 = new Pose(0, 0, 0);
        Pose pose1 = new Pose(DIST, 0, 0);

        drive.setPose(pose0);

        while(opModeIsActive()){
            diego.driveTo(pose1, mProf, 1.0, this::updateAll);
            maxSpeed = tempMaxSpeed;
            tempMaxSpeed = 0;
            diego.holdPoseAsync(1000, pose1, this::updateAll);
            diego.driveTo(pose0, mProf, 1.0, this::updateAll);
            maxSpeed = tempMaxSpeed;
            tempMaxSpeed = 0;
            diego.holdPoseAsync(1000, pose0, this::updateAll);
        }

    }

    private void updateAll(){
        Pose vel = drive.getVelocity();
        speed = Math.hypot(vel.getX(), vel.getY());
        if (speed > tempMaxSpeed) tempMaxSpeed = speed;
        panelsTelemetry.addData("speed", speed);
        panelsTelemetry.addData("maxSpeed", maxSpeed);
        if (gamepad1.aWasPressed()){
            drive.setPIDFCoefficients(NEXT_P, 0, NEXT_D, NEXT_F);
        }
        PIDFCoefficients pidf = drive.getPIDFCoefficients();
        panelsTelemetry.debug("pidf: ", String.format("P %.4f  I %.4f  D %.4f  F %.4f",
                pidf.p, pidf.i, pidf.d, pidf.f));
        panelsTelemetry.update(telemetry);

    }

}
