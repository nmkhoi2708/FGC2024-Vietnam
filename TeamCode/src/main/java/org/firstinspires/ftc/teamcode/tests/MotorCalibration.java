package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Constants.BASE.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.Dataflow;

@Autonomous(name="MotorCalibration")
public class MotorCalibration extends LinearOpMode {
    private DcMotorEx motor;
    Dataflow dataflow = new Dataflow(telemetry);
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for the motor and gearing
    private static final double GEAR_RATIO = 13.0321;
    private static final double MOTOR_CPR = 28;  // Counts per Revolution of the HD Hex Motor
    private static final int TARGET_POSITION = (int) (MOTOR_CPR * GEAR_RATIO);

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor"); //initialize the motor
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            revolutionTest(1); //run 1 round continously
        }
    }

    public void revolutionTest(double revo) { //test the encoder by running the motor continously
        int newTarget;
        if (opModeIsActive()) {
            newTarget = motor.getCurrentPosition() + (int)(TARGET_POSITION * revo); //calculate the new target
            motor.setTargetPosition(newTarget);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            runtime.reset();
            motor.setPower(0.1); //set motor power
            while (opModeIsActive() && motor.isBusy()) {
                dataflow.addToAll(new String[]{"Running to", "Currently at", "Velocity", "Current"},    //send telemetry data about the target, cur position, velcity and electrical current
                                                newTarget, motor.getCurrentPosition(), motor.getVelocity(), motor.getCurrent(CurrentUnit.MILLIAMPS));
            }
            motor.setPower(0); //stop the motor
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sleep(2500);  // Pause for a while before ending the test
        }
    }
}
