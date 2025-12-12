package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    DcMotorEx motor;
    double previousPower = -1;
    public Lift(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "lift");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
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
