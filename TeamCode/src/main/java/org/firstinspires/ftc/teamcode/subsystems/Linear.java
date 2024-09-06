package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Linear {
    private DcMotorEx upLinear; //create an obj for linear motors
    private DcMotorEx downLinear; 
    private DcMotorEx middleLinear; 
    private Servo servo1; //create an obj for the servos
    private Servo servo2; 
    private final HardwareMap hardwareMap; //create an obj for the usage of HardwareMap

    public Linear(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap; //create an access for OpModes
    }

    public void init() {
        upLinear = hardwareMap.get(DcMotorEx.class, "upLinear"); //connect with the hardware motors
        downLinear = hardwareMap.get(DcMotorEx.class, "downLinear");
        middleLinear = hardwareMap.get(DcMotorEx.class, "middleLinear"); 
        servo1 = hardwareMap.get(Servo.class, "servo1"); //connect with the linear servos
        servo2 = hardwareMap.get(Servo.class, "servo2"); 

        upLinear.setDirection(DcMotorSimple.Direction.REVERSE); //set the direction of up linear motor to REVERSE
        downLinear.setDirection(DcMotorSimple.Direction.FORWARD); //set the direction of the down linear motor to FORWARD

        servo1.setDirection(Servo.Direction.FORWARD); //set servos direction
        servo2.setDirection(Servo.Direction.REVERSE);

        upLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //set linear motors runMode
        downLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        middleLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //set linear motors Zero Power Behaviour to BRAKE
        downLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setUpLinear(double pow) { //set up linear motor power
        upLinear.setPower(pow);
    }

    public void setDownLinear(double pow) { //set down linear motor power
        downLinear.setPower(pow);
    }

    public void setMiddleLinear(double pow) { //set middle linear motor power
        middleLinear.setPower(pow);
    }

    public void setAllLinear(double pow) { //set all linear motors power
        setDownLinear(pow);
        setUpLinear(pow);
    }

    public void setLinearServo(double pos) { //set linear servo position
        servo2.setPosition(pos);
        servo1.setPosition(pos);
    }
}
