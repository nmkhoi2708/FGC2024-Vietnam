package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Linear servo testing")
@Disabled
public class LinearServo extends LinearOpMode {
    private Servo servo;
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);
        double current_pos = servo.getPosition();
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Current position", current_pos);
            servo.setPosition(1.0);
        }
    }
}
