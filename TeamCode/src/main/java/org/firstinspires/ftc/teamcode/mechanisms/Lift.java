package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    DcMotorEx motor;

    Servo leftBob;
    Servo rightBob;

    double previousPower = -1;

    final double LEFT_BOB_END = 0.2502;

    final double LEFT_BOB_START = 0.8411;

    final double RIGHT_BOB_START = 0.9816;

    final double RIGHT_BOB_END = 0.2857;

    public Lift(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "lift");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBob = hardwareMap.get(Servo.class, "left_bob");
        rightBob = hardwareMap.get(Servo.class, "right_bob");
        lockLift();
    }

    public void lockLift(){
        leftBob.setPosition(LEFT_BOB_START);
        rightBob.setPosition(RIGHT_BOB_START);
    }

    public void releaseLift(){
        leftBob.setPosition(LEFT_BOB_END);
        rightBob.setPosition(RIGHT_BOB_END);
    }

    public void holdPosition(){
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) return;
        int pos = motor.getCurrentPosition();
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public void setPower(double power){
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public double getPower(){
        return motor.getPower();
    }

    public void togglePower(){
        double p = getPower();
        double newPower;
        if (Math.abs(p) < 0.001){
            newPower = previousPower < 0? 1: -1;
        } else {
            newPower = 0 ;
        }
        previousPower = p;
        setPower(newPower);
    }

}
