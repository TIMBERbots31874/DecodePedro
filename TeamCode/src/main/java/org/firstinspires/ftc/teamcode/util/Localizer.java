package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public interface Localizer {

    void update();

    Pose getPose();

    Pose getVelocity();

    void setPose(Pose pose);

}
