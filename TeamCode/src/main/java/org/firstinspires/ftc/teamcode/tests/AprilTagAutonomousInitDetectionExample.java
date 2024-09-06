package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;

@Autonomous(name = "AprilTag Detection Example")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode { //to test apriltag detection

    private AprilTagCamera aprilTagCamera; //create obj for the camera and gamepad usage
    private Gamepad gamepad;
    @Override
    public void runOpMode() {
        aprilTagCamera = new AprilTagCamera(this, gamepad); 
        aprilTagCamera.init(); //initialize the camera

        aprilTagCamera.trackAprilTag(); //return telemetry datas whether the camera has or hasn't detected an apriltag 

        waitForStart();

        while(opModeIsActive()) {
            sleep(20);
        }
    }
}

