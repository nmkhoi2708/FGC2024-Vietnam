package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.utils.Dataflow;

public class Robot {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Drivebase driveBase;
    private Linear linear;
    private IMU imu;
    private Telemetry telemetry;
    private boolean autoAlign = false;
    private Dataflow dataflow;

    public Robot(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.driveBase = new Drivebase(opMode);
        this.linear = new Linear(opMode);
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.dataflow = new Dataflow(this.telemetry);
    }

    public void init() {
        driveBase.init();
    }

    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;

        double leftPower = Range.clip(leftY, -1.0, 1.0);
        double rightPower = Range.clip(rightY, -1.0, 1.0);

        driveBase.setMotorsPower(leftPower, rightPower);

        driveBase.setHorizontalMove(-gamepad1.left_trigger);
        driveBase.setHorizontalMove(gamepad1.right_trigger);


        if (gamepad1.x) {
            driveBase.boost();
        }

        dataflow.sendDatasToTelemetry(new String[]{"LeftFront:", "LeftBack:", "RightFront:", "RightBack:"},
                driveBase.getLeftPower(), driveBase.getRightPower());
    }
}