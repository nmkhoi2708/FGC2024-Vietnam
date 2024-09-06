package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoSystems;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.utils.Dataflow;

public class Robot {
    private Gamepad gamepad1; //create objects for robot control 
    private Gamepad gamepad2;
    private Drivebase driveBase;
    private Linear linear;
    private Telemetry telemetry;
    private Dataflow dataflow;
    private AutoSystems autoSystems;

    private enum RobotState { IDLE, TURNING, MANUAL_CONTROL }
    private RobotState currentState;

    public Robot(LinearOpMode linearOpMode) { //create an access for LinearOpModes
        this.telemetry = linearOpMode.telemetry; 
        this.driveBase = new Drivebase(linearOpMode);
        this.linear = new Linear(linearOpMode);
        this.gamepad1 = linearOpMode.gamepad1;
        this.gamepad2 = linearOpMode.gamepad2;
        this.dataflow = new Dataflow(this.telemetry);
        this.autoSystems = new AutoSystems(linearOpMode);
        this.currentState = RobotState.IDLE;//set to the idle state
    }

    public void init() { //initialize
        driveBase.init();
        autoSystems.init();
        linear.init();
    }

    public void loop(LinearOpMode linearOpMode) {
        while (linearOpMode.opModeIsActive()) {
            switch (currentState) { //set the mode ( idle / auto / manual )
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
            dataflow.addToAll(new String[]{ "LeftBack:",
                                            "RightBack:",
                                            "Current State:"},
                                            driveBase.getLeftPower(),
                                            driveBase.getRightPower(),
                                            currentState);
            dataflow.sendDatas(); //send telemetry datas
        }
    }

    private void idleState() { 
        if (gamepad1.circle) {
            currentState = RobotState.TURNING; //turn to auto mode if the circle button is pressed
        } else if (gamepad1.start) {
            currentState = RobotState.MANUAL_CONTROL; //turn to manual mode if the start button is pressed
        }
    }

    private void turningState() { //turn to the desired heading then switch to manual mode and rumble the gamepad
        autoSystems.turnToHeading(90);
        currentState = RobotState.MANUAL_CONTROL;
        gamepad1.rumble(100);
    }

    private void manualControlState() {
        //Joy-stick
        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;
        double leftPower = Range.clip(leftY, -1.0, 1.0);
        double rightPower = Range.clip(rightY, -1.0, 1.0);

        //Triggers and bumpers
        driveBase.setMotorsPower(leftPower, rightPower);
        driveBase.setHorizontalMove(gamepad1.left_trigger-gamepad1.right_trigger);

        //Dpad
        if (gamepad1.dpad_up) {
            linear.setAllLinear(1.0);
        } else if (gamepad1.dpad_down) {
            linear.setAllLinear(-1.0);
        } else if (gamepad1.dpad_left) {
            linear.setMiddleLinear(1.0);
        } else if (gamepad1.dpad_right) {
            linear.setMiddleLinear(-1.0);
        } else {
            linear.setAllLinear(0);
            linear.setMiddleLinear(0);
        }

        //Color pad
        if (gamepad1.circle) {
            currentState = RobotState.TURNING;
        }

        if(gamepad1.triangle) {
            linear.setLinearServo(1.0);
        } else if(gamepad1.cross) {
            linear.setLinearServo(0.05);
        } else if(gamepad1.square) {
            linear.setLinearServo(0.35);
        }

    }
}
