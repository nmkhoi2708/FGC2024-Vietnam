package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Linear;

public class Robot {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private Drivebase driveBase;
    private Linear linear;
    private IMU imu;

    private boolean autoAlign = false;

    public Robot(OpMode opMode) {
        driveBase = new Drivebase(opMode);
        linear = new Linear(opMode);

        telemetry = opMode.telemetry;
        gamepad2 = opMode.gamepad2;
        gamepad1 = opMode.gamepad1;
    }

    public void init() {
        driveBase.init();
//        linear.init();
    }

    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;

        double leftPower = Range.clip(leftY, -1.0, 1.0);
        double rightPower = Range.clip(rightY, -1.0, 1.0);

        driveBase.setMotorPower(leftPower, rightPower);

        if(gamepad1.left_trigger > 0) {
            driveBase.setHorizontalMove(-gamepad1.left_trigger);
        } else if(gamepad1.right_trigger > 0) {
            driveBase.setHorizontalMove(gamepad1.right_trigger);
        } else {
            driveBase.setHorizontalMove(0);
        }

        if(gamepad1.x) {
            driveBase.boost();
        }
//        if (gamepad1.left_bumper) {
//            linear.setFrontLinear(1.0);
//        } else if (gamepad1.left_trigger > 0.5) {
//            linear.setFrontLinear(-1.0);
//        } else {
//            linear.setFrontLinear(0.0);
//        }
//
//        if (gamepad1.right_bumper) {
//            linear.setBackLinear(1.0);
//        } else if (gamepad1.right_trigger > 0.5) {
//            linear.setBackLinear(-1.0);
//        } else {
//            linear.setBackLinear(0.0);
//        }
//
//        if (gamepad1.dpad_down) {
//            linear.setMiddleLinear(-0.4);
//        } else if (gamepad1.dpad_up) {
//            linear.setMiddleLinear(0.4);
//        } else {
//            linear.setMiddleLinear(0.0);
//        }
//
//        if(gamepad1.circle) {
//            linear.setLinearServo(1.0);
//        } else if(gamepad1.square) {
//            linear.setLinearServo(-1.0);
//        } else {
//            linear.setLinearServo(0);
//        }
    }
}
