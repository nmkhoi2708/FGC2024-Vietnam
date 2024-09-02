package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Constants.FIELD.HORIZONTAL_AUTO_REQUIREMENT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoSystems;

@Autonomous(name="HorizontalAlign", group="Robot")
public class HorizontalAlign extends LinearOpMode {
    private AutoSystems autoSystems;
    @Override
    public void runOpMode() {
        autoSystems = new AutoSystems(this);
        autoSystems.init();

        waitForStart();
        autoSystems.horizontalMove(1);
    }
}
