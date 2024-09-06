package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import java.util.Arrays;
import java.util.List;

public class Drivebase {
    private DcMotorEx leftFront; //motors obj declaration 
    private DcMotorEx rightFront; 
    private DcMotorEx leftBack; 
    private DcMotorEx rightBack; 
    public DcMotorEx middleWheel;
    private final HardwareMap hardwareMap; //declare an obj for hardwaremap usage
    private double speed = NORMAL_DRIVE; //set the speed ratio to normal
    public List<DcMotorEx> motors, leftMotors, rightMotors; //declare lists for motors


    public Drivebase(LinearOpMode linearOpMode) {
        this.hardwareMap = linearOpMode.hardwareMap; //provide an access for OpModes
    }

    public void init(){ //code in this function will run in the init phase
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront"); //connect with the hardware motors
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");  
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack"); 
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack"); 
        middleWheel = hardwareMap.get(DcMotorEx.class, "middleHex");

        motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront); //declare array elements
        rightMotors = Arrays.asList(rightFront, rightBack); 
        leftMotors = Arrays.asList(leftFront, leftBack); 

        setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set runMode for motors 
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set the ZeroPowerBehavior of motors to BRAKE
        setDirection(DcMotorSimple.Direction.REVERSE, rightMotors); // set the direction of right motors to REVERSE
    }

    public void setMotorsPower(double leftPower, double rightPower){ //set motors power
        leftFront.setPower(leftPower * speed);
        leftBack.setPower(leftPower * speed);
        rightFront.setPower(rightPower * speed);
        rightBack.setPower(rightPower * speed);
    }

    public void setHorizontalMove(double hex_power) { //set middle motor power
        double power;
        power = Range.clip(hex_power, -1.0, 1.0); //clip the hex_power if hex_power is lower than -1.0 or greater than 1.0
        middleWheel.setPower(power);
    }

    public int getPosition(DcMotor motor) {
        return motor.getCurrentPosition(); //return the encoder value on the specific motor
    }

    public void boost() { //change the speed ratio status
        if(speed == NORMAL_DRIVE) { 
            speed = BOOST_DRIVE; //if the speed ratio status is normal, set it to boosted mode
        } else {
            speed = NORMAL_DRIVE; //if the speed ratio is boosted, set it back to normal 
        }
    }

    public double getPower(DcMotorEx motorEx) {
        return motorEx.getPower(); //return the motor power
    }

    public double getLeftPower() {
        return getPower(leftBack); //return left back motor power
    }

    public double getRightPower() {
        return getPower(rightBack); //return right back motor power
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode); //set motors runMode
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior); //set motors ZeroPowerBehavior
        }
    }

    public void setDirection(DcMotorSimple.Direction Direction, List<DcMotorEx> sideMotors) {
        for (DcMotorEx motor : sideMotors) {
            motor.setDirection(Direction); //set side motors Direction
        }
    }
}
