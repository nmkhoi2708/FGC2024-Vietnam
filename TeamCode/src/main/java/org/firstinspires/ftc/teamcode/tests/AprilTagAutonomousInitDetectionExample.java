package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;

@Autonomous(name = "AprilTag Detection Example")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode {

    private AprilTagCamera aprilTagCamera;
    private Gamepad gamepad;
    @Override
    public void runOpMode() {
        aprilTagCamera = new AprilTagCamera(this, gamepad);
        aprilTagCamera.init();

        aprilTagCamera.trackAprilTag();

        waitForStart();

        while(opModeIsActive()) {
            sleep(20);
        }
    }
}

