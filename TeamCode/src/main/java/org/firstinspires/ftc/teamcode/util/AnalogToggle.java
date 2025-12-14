package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

public class AnalogToggle {
    Supplier<Float> supplier;
    double pressedThreshold;
    double releasedThreshold;
    boolean pressed = false;

    public AnalogToggle(Supplier<Float> supplier, double pressedThreshold, double releasedThreshold){
        this.pressedThreshold = pressedThreshold;
        this.releasedThreshold = releasedThreshold;
        this.supplier = supplier;
    }

    public boolean update(){
        double value = supplier.get();
        boolean active, inactive;
        if (pressedThreshold > releasedThreshold){
            active = value > pressedThreshold;
            inactive = value < releasedThreshold;
        } else {
            active = value < pressedThreshold;
            inactive = value > releasedThreshold;
        }
        boolean result = active && !pressed;
        pressed = active || (pressed && !inactive);
        return result;
    }

}
