package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUHandler {
    private IMU imu; //create an obj for the imu
    private final HardwareMap hardwareMap; //create an obj for hardwareMap usage

    public IMUHandler(LinearOpMode linearOpMode) {
        this.hardwareMap = linearOpMode.hardwareMap; //create an access for OpModes
    }

    public void init() {
        imu = hardwareMap.get(IMU.class, "imu"); //connect with the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; //declare the IMU direction
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection); //direction of the REV control hub on the robot

        imu.initialize(new IMU.Parameters(orientationOnRobot)); //create a new imu parameters
    }

    public double getHeading() { //get the current heading of the robot in degree
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() { //reset the heading back to 0 
        imu.resetYaw();
    }
}
