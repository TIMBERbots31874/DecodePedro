package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class SpinnyJeff {

    Servo servo;
    RevColorSensorV3 color;

    double[] allPositions = {0.1407, 0.2188, 0.2924, 0.3588, 0.4354, 0.5116};
    int currentIndex = 0;
    boolean movingForward = true;
    int MAX_COLOR_VALUE = 360;
    

    public SpinnyJeff(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "spinner_servo");
//        color = hardwareMap.get(RevColorSensorV3.class, "color");
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

//    public float[] getHSV(){
//        int red = color.red() * 255 / MAX_COLOR_VALUE;
//        int green = color.green() * 255 / MAX_COLOR_VALUE;
//        int blue = color.blue() * 255 / MAX_COLOR_VALUE;
//        float[] hsv = new float[3];
//        Color.RGBToHSV(red, green ,blue, hsv);
//        return hsv;
//    }

}
