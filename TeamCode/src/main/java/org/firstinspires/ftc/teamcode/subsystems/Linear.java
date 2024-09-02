package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Linear {
    private DcMotorEx upLinear;
    private DcMotorEx downLinear;
    private DcMotorEx middleLinear;
    private CRServo servo1;
    private CRServo servo2;
    private final HardwareMap hardwareMap;

    public Linear(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        upLinear = hardwareMap.get(DcMotorEx.class, "upLinear");
        downLinear = hardwareMap.get(DcMotorEx.class, "downLinear");
        middleLinear = hardwareMap.get(DcMotorEx.class, "middleLinear");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        upLinear.setDirection(DcMotorSimple.Direction.FORWARD);
        downLinear.setDirection(DcMotorSimple.Direction.REVERSE);

        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);

        upLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        downLinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        middleLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        downLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setUpLinear(double pow) {
        upLinear.setPower(pow);
    }

    public void setDownLinear(double pow) {
        downLinear.setPower(pow);
    }

    public void setMiddleLinear(double pow) {
        middleLinear.setPower(pow);
    }

    public void setAllLinear(double pow) {
        setDownLinear(pow);
        setUpLinear(pow);
    }

    public void setLinearServo(double pow) {
        servo2.setPower(pow);
        servo1.setPower(pow);
    }
}
