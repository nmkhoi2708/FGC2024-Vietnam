package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Linear {
    private DcMotorEx frontLinear;
    private DcMotorEx backLinear;
    private DcMotorEx middleLinear;
    private CRServo servo1;
    private CRServo servo2;
    private final HardwareMap hardwareMap;

    public Linear(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        frontLinear = hardwareMap.get(DcMotorEx.class, "frontLinear");
        backLinear = hardwareMap.get(DcMotorEx.class, "backLinear");
        middleLinear = hardwareMap.get(DcMotorEx.class, "middleLinear");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);

        frontLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        middleLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setFrontLinear(double pow) {
        frontLinear.setPower(pow);
    }

    public void setBackLinear(double pow) {
        backLinear.setPower(pow);
    }

    public void setMiddleLinear(double pow) {
        middleLinear.setPower(pow);
    }

    public void setAllLinear(double pow) {
        setBackLinear(pow);
        setFrontLinear(pow);
        setMiddleLinear(pow);
    }

    public void setLinearServo(double pow) {
        servo2.setPower(pow);
        servo1.setPower(pow);
    }
}
