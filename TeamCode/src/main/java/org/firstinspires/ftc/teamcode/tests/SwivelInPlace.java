package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoSystems;

@Autonomous(name="Swivel In Place", group="Robot")
public class SwivelInPlace extends LinearOpMode {
    private AutoSystems autoSystems; //create obj for autosystems
    @Override
    public void runOpMode() {
        autoSystems = new AutoSystems(this); //initialize
        autoSystems.init();

        waitForStart();
        autoSystems.turnToHeading(90); //turn to a 90 degree heading returned by the imu
    }
}
