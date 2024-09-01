package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoSystems;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.IMUHandler;
import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.utils.Dataflow;

public class Robot {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Drivebase driveBase;
    private Linear linear;
    private Telemetry telemetry;
    private Dataflow dataflow;
    private AutoSystems autoSystems;

    private enum RobotState { IDLE, TURNING, MANUAL_CONTROL }
    private RobotState currentState;

    public Robot(LinearOpMode linearOpMode) {
        this.telemetry = linearOpMode.telemetry;
        this.driveBase = new Drivebase(linearOpMode);
        this.linear = new Linear(linearOpMode);
        this.gamepad1 = linearOpMode.gamepad1;
        this.gamepad2 = linearOpMode.gamepad2;
        this.dataflow = new Dataflow(this.telemetry);
        this.autoSystems = new AutoSystems(linearOpMode);
        this.currentState = RobotState.IDLE;
    }

    public void init() {
        driveBase.init();
        autoSystems.init();
    }

    public void loop(LinearOpMode linearOpMode) {
        while (linearOpMode.opModeIsActive()) {
            switch (currentState) {
                case IDLE:
                    idleState();
                    break;
                case TURNING:
                    turningState();
                    break;
                case MANUAL_CONTROL:
                    manualControlState();
                    break;
            }
            dataflow.addToAll(new String[]{"LeftBack:", "RightBack:", "Current State:"}, driveBase.getLeftPower(), driveBase.getRightPower(), currentState);
            dataflow.sendDatas();
        }
    }

    private void idleState() {
        if (gamepad1.circle) {
            currentState = RobotState.TURNING;
        } else if (gamepad1.start) {
            currentState = RobotState.MANUAL_CONTROL;
        }
    }

    private void turningState() {
        autoSystems.turnToHeading(90);
        currentState = RobotState.MANUAL_CONTROL;
    }

    private void manualControlState() {
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
        if (gamepad1.circle) {
            currentState = RobotState.TURNING;
        }
        if (gamepad1.triangle) {
            currentState = RobotState.IDLE;
        }

    }
}
