package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous(name="Swivel In Place", group="Robot")
public class SwivelInPlace extends OpMode {
    private Drivebase drivebase;

    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    @Override
    public void init() {
        drivebase = new Drivebase(this);
        drivebase.init();
    }

    @Override
    public void loop() {

    }
}