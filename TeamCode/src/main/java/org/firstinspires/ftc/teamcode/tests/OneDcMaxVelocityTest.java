package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Dataflow;

@TeleOp
@Disabled
public class OneDcMaxVelocityTest extends LinearOpMode {
    Dataflow dataflow; //create objects for the motor and variables for calculating max velocity
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "DcHex");//initialize the motor
        waitForStart();

        while (opModeIsActive()) { 
            currentVelocity = motor.getVelocity(); //get current velocity
            if (currentVelocity > maxVelocity) { //if the curVelocity > max, set a new max velocity
                maxVelocity = currentVelocity;
            }
            dataflow.addToAll(new String[]{"Current velocity", "Maximum velocity"}, currentVelocity ,maxVelocity);//send telemetry data
            dataflow.sendDatas();
        }
    }
}
