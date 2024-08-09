package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous(name="Swivel In Place", group="Robot")
public class SwivelInPlace extends LinearOpMode {
    private Drivebase drivebase;
    @Override
    public void runOpMode() {
        // Initialize the Drivebase
        drivebase = new Drivebase(this);
        drivebase.init();

        waitForStart();

        // Autonomous sequence
        drivebase.turnToHeading(90, this);
        drivebase.holdHeading(90, 0.5); // Hold the heading for 0.5 seconds
    }
}
