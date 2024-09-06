package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Linear servo testing")
@Disabled
public class LinearServo extends LinearOpMode {
    private Servo servo; //create an obj for the servo usage
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo"); //connect with the hardware servo
        servo.setDirection(Servo.Direction.FORWARD);
        double current_pos = servo.getPosition(); //get current position of the servo
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Current position", current_pos); //send telemetry data of servo position
            servo.setPosition(1.0); //set servo position
        }
    }
}
