package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Constants.FIELD.HORIZONTAL_AUTO_REQUIREMENT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoSystems;

@Autonomous(name="HorizontalAlign", group="Robot")
public class HorizontalAlign extends LinearOpMode {
    private AutoSystems autoSystems; //create an obj for the autosystem usage
    @Override
    public void runOpMode() {
        autoSystems = new AutoSystems(this); //declare the Autosystem class usage
        autoSystems.init(); //initialize

        waitForStart();
        autoSystems.horizontalMove(1); //set the target to 1 and move to the target position using the middle wheel
    }
}
