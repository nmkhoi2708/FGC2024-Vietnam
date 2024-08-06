package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

public class Drivebase {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx middleWheel;
    BNO055IMU imu;
    private final HardwareMap hardwareMap;

    private double speed = NORMAL_DRIVE;

    public Drivebase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        middleWheel = hardwareMap.get(DcMotorEx.class, "middleHex");

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorPower(double leftPower, double rightPower){
        leftFront.setPower(leftPower * speed);
        leftBack.setPower(leftPower * speed);
        rightFront.setPower(rightPower * speed);
        rightBack.setPower(rightPower * speed);
    }

    public void setHorizontalMove(double hex_power) {
        double power;
        if(abs(hex_power) >= 1.0) {
            power = Range.clip(hex_power, -1.0, 1.0);
        } else {
            power = 0;
        }
        middleWheel.setPower(power);
    }

    public int getPosition(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void boost() {
        if(speed == NORMAL_DRIVE) {
            speed = BOOST_DRIVE;
        } else {
            speed = NORMAL_DRIVE;
        }
    }

    public double getPower(DcMotorEx motorEx) {
        return motorEx.getPower();
    }

    public double getLeftPower() {
        return getPower(leftBack);
    }

    public double getRightPower() {
        return getPower(rightBack);
    }

}
