package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class SpinnyJeff {

    Servo servo;

    double[] allPositions = {0.0358, 0.1063, 0.1829, 0.2549, 0.326, 0.4026, 0.4754, 0.5538,
        0.6295, 0.6976, 0.769, 0.8407, 0.9131, 0.9827};
    int currentIndex = 0;
    boolean movingForward = true;
    

    public SpinnyJeff(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "spinner_servo");
        setIndex(0);
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
