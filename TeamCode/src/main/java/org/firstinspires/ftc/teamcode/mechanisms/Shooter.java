package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    DcMotorEx rightMotor, leftMotor;

    double kickerOut = 0.0973; //was 0.0973, 1.099
    double kickerIn = 0.27;

    double targetSpeed = 0;

    public final double FEED_FORWARD_COEFF =  0.9 / 2500.0;
    public final double PROPORTIONATE_COEFF = 1.5 / 1000.0;

    public final double DIFF_COEFF = 1.5 / 2000.0;

    Servo kicker;

    VoltageSensor voltageSensor;

    public Shooter(HardwareMap hardwareMap){
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_shooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_shooter");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        kicker = hardwareMap.get(Servo.class, "kicker");
        releaseKicker();

        voltageSensor = hardwareMap.getAll(VoltageSensor.class).get(0);

    }

    public void setSpeed(double speed){
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
    }

    public double getRightSpeed() {
        return rightMotor.getVelocity();
    }

    public double getLeftSpeed() {
        return leftMotor.getVelocity();
    }

    public double getRightPower(){
        return rightMotor.getPower();
    }

    public double getLeftPower(){
        return leftMotor.getPower();
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = targetSpeed;
    }

    public void update(){
        double leftSpeed = getLeftSpeed();
        double rightSpeed = getRightSpeed();
        double avg = (leftSpeed + rightSpeed) / 2.0;
        double diff = leftSpeed - rightSpeed;

        double voltage = voltageSensor.getVoltage();

        double leftPower = targetSpeed * FEED_FORWARD_COEFF
                + (targetSpeed - avg) * PROPORTIONATE_COEFF
                - diff * DIFF_COEFF;

        leftPower *= 13.0 / voltage;

        leftPower = Range.clip(leftPower, 0, 1);

        double rightPower = targetSpeed * FEED_FORWARD_COEFF
                + (targetSpeed - avg) * PROPORTIONATE_COEFF
                + diff * DIFF_COEFF;

        rightPower *= 13.0 / voltage;

        rightPower = Range.clip(rightPower, 0, 1);



        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
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
