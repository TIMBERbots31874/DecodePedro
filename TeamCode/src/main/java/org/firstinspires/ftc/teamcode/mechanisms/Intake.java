package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotorEx motor;

    public enum State{STOPPED, FORWARD, REVERSE}

    State state = State.STOPPED;

    public static final double FULL_SPEED = 1;

    public Intake(HardwareMap hardwaremap){
        //* what is needed to initalize the hardware we are using

        motor = hardwaremap.get(DcMotorEx.class, "intake_motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void setState(State state){
        this.state = state;

        switch(state){
            case STOPPED:
                motor.setPower(0);
                break;
            case FORWARD:
                motor.setPower(FULL_SPEED);
                break;
            case REVERSE:
                motor.setPower(-FULL_SPEED);
        }
    }

    public State getState(){
        return state;
    }

    public State state(){
        return state;
    }

}
