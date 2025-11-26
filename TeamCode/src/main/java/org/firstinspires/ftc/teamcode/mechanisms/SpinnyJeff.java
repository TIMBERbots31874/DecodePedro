package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpinnyJeff {

    Servo servo;

    enum Hole{A,B, C}
    double[] aPositions = {0.1, 0.3, 0.5, 0.7, 0.9};
    double[] bPositions = {0.17, 0.37, 0.57, 0.77, 0.97};
    double[] cPositions = {0.03, 0.23, 0.43, 0.63, 0.83};
    double[] allPositions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    int currentIndex = 0;
    boolean movingForward = true;
    

    public SpinnyJeff(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "spinner_servo");
        servo.setPosition(allPositions[0]);
    }

    public void setPosition(double Fred){
        servo.setPosition(Fred);
    }

    public void setIndex(int index){
        currentIndex = index;
        servo.setPosition(allPositions[index]);
    }

    public void moveForward(){
        if (currentIndex == allPositions.length - 1) return;
        setIndex(currentIndex + 1);
    }

    public void moveReverse(){
        if (currentIndex == 0) return;
        setIndex(currentIndex - 1);
    }

    public void moveNext(){
        if (movingForward) movingForward = currentIndex < allPositions.length -1;
        else movingForward = currentIndex == 0;
        if (movingForward) moveForward();
        else moveReverse();
    }

}
