package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.HashMap;

//This is a blueprint for making objects
public class Motion {

    //These are the instance variables-- when we use the constructor method to create an object whose
    //they are instance as they do not have the words static before them and because they are
    //declared in the class and not inside a method
    DcMotorEx fL, bL, fR, bR;

    OpMode opMode;

    GoBildaPinpointDriver odo;

    VoltageSensor voltageSensor;


    final double FORWARD_TICKS_PER_INCH = 42.3;
    final double STRAFE_TICKS_PER_INCH = 49.8;
    final double TICKS_PER_RADIAN = 631;
    final double MAX_TICKS_PER_SEC = 2400;

        //This is a constructor method
    public Motion(OpMode opMode) {
        this.opMode = opMode;

        //This is the definitions of our drive motors.
        fL = opMode.hardwareMap.get(DcMotorEx.class, "front_left_motor");
        bL = opMode.hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fR = opMode.hardwareMap.get(DcMotorEx.class, "front_right_motor");
        bR = opMode.hardwareMap.get(DcMotorEx.class,"back_right_motor");

        //for mecanum drive-- positive power makes robot go forward with relative power- left motors need to be negated
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This tells the program we are using the encoders on the motors
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = opMode.hardwareMap.getAll(VoltageSensor.class).get(0);

        setCompensatedPIDFCoefficients(10, 0.5, 0, 13);

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(6.16, -6.65, DistanceUnit.INCH);
        odo.setYawScalar(1.00167);
        odo.resetPosAndIMU();
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

    }

    //This is a method that will return the position of the robot on the field (instance method)
    public Pose getPose() {
        Pose2D pose2D = odo.getPosition();
        Pose result = new Pose(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
        return result;
    }

    public Pose getVelocity(){
        return new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }


    public double getCumulativeHeading(){
        return odo.getHeading(AngleUnit.RADIANS);
    }

    public void setPose(Pose pose) {
        Pose2D pose2d = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading());
        //called the method and passed the parameter in to set the position (where the robot is)
        odo.setPosition(pose2d);
    }

    public void updateOdometry() {
        odo.update();
    }

    public void setDrivePower(double px, double py, double pa) {
        double pBL = px + py - pa;
        double pFL = px - py - pa;
        double pFR = px + py + pa;
        double pBR = px - py + pa;
        //power of motor -1 <= power <=1`

        double max = 1;

        //assignment of the max variable and calls the math.max function that will give back the largest number
        max = Math.max(max, Math.abs(pBL));
        max = Math.max(max, Math.abs(pFL));
        max = Math.max(max, Math.abs(pFR));
        max = Math.max(max, Math.abs(pBR));

        if (max >1) {
            pBL /= max; //Java short hand for pbl = pbl/max
            pFL /= max;
            pFR /= max;
            pBR /= max;
        }

        bL.setPower(pBL);
        fL.setPower(pFL);
        fR.setPower(pFR);
        bR.setPower(pBR);

    }

    public void setDriveSpeed(double vx, double vy, double va){
        double px = vx * FORWARD_TICKS_PER_INCH / MAX_TICKS_PER_SEC;
        double py = vy * STRAFE_TICKS_PER_INCH / MAX_TICKS_PER_SEC;
        double pa = va * TICKS_PER_RADIAN / MAX_TICKS_PER_SEC;
        setDrivePower(px, py, pa);
    }

    public void setMotorPowers(double pBL, double pFL, double pFR, double pBR){
        bL.setPower(pBL);
        fL.setPower(pFL);
        fR.setPower(pFR);
        bR.setPower(pBR);
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        bL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        fL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        fR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setCompensatedPIDFCoefficients(double p, double i, double d, double f){
        double v = voltageSensor.getVoltage();
        setPIDFCoefficients(p*13.5/v, i*13.5/v, d*13.5/v, f*13.5/v);
    }

    public PIDFCoefficients getPIDFCoefficients(){
        return bL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public HashMap<String,PIDFCoefficients> getAllPIDFCoefficients(){
        HashMap<String,PIDFCoefficients> result = new HashMap<>();
        result.put("bL", bL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("fL", fL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("fR", fR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("bR", bR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        return result;
    }

    public double getDriveCurrent(){
        return Math.abs(bL.getCurrent(CurrentUnit.AMPS)) + Math.abs(fL.getCurrent(CurrentUnit.AMPS))
                + Math.abs(fR.getCurrent(CurrentUnit.AMPS)) + Math.abs(bR.getCurrent(CurrentUnit.AMPS));
    }

}
