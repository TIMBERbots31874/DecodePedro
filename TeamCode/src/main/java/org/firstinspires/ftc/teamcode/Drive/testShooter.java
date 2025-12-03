package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public abstract class testShooter extends LinearOpMode {
    DcMotorEx rightMotor, leftMotor;
    double kickerOut = 0;
    double kickerIn = 1;
    Servo kicker;
    public testShooter(HardwareMap hardwareMap){
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_shooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_shooter");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker = hardwareMap.get(Servo.class, "kicker");
    }

    public void setSpeed(double speed){
        rightMotor.setPower(speed);
         leftMotor.setPower(speed);
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
