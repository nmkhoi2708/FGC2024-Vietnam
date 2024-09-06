package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Robot Autonomous", group = "Robot")
public class Main extends LinearOpMode {
    private Robot robot; //create an object for robot class usage

    @Override
    public void runOpMode() {
        robot = new Robot(this);//initialize
        robot.init();

        waitForStart();

        if (opModeIsActive()) {
            robot.loop(this); //run the robot loop
        }
    }
}
