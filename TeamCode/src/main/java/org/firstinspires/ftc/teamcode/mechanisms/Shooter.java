package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Shooter {
    DcMotorEx rightMotor, leftMotor;

    double kickerOut = 0.0973;
    double kickerIn = 0.27;

    Servo kicker;
    public Shooter(HardwareMap hardwareMap){
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_shooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_shooter");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker = hardwareMap.get(Servo.class, "kicker");
        releaseKicker();
    }

    public void setSpeed(double speed){
        rightMotor.setPower(1);
        leftMotor.setPower(1);
    }

    public void setKicker(double pos){
        kicker.setPosition(pos);
    }

    public void engageKicker(){
        setKicker(kickerIn);
    }

    public void releaseKicker(){
        setKicker(kickerOut);
    }

    public void shoot(){
        engageKicker();
        ElapsedTime elapsedTime = new ElapsedTime();

        while (elapsedTime.milliseconds() < 300) continue;
        releaseKicker();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < 300) continue;
    }

}
