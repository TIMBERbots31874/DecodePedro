package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class SpinnyJeff {

    Servo servo;

    double[] allPositions = {0.1407, 0.2188, 0.2924, 0.3588, 0.4354, 0.5116};
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

    public int getIndex(){return currentIndex;}
    public void moveForward(){
//        if (currentIndex == allPositions.length - 1) return;
        setIndex(currentIndex + 1);
    }

    public void moveReverse(){
//        if (currentIndex == 0) return;
        setIndex(currentIndex - 1);
    }

    public void moveNext(){
        if (movingForward) movingForward = currentIndex < allPositions.length -1;
        else movingForward = currentIndex == 0;
        if (movingForward) moveForward();
        else moveReverse();
    }

}
