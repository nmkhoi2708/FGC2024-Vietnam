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
    private static final double GEAR_RATIO = 16.0;
    private static final double MOTOR_CPR = 23;  // Counts per Revolution of the HD Hex Motor
    private static final int TARGET_POSITION = (int) (MOTOR_CPR * GEAR_RATIO);

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            revolutionTest(1);
        }
    }

    public void revolutionTest(double revo) {
        int newTarget;
        if (opModeIsActive()) {
            newTarget = motor.getCurrentPosition() + (int)(TARGET_POSITION * revo);
            motor.setTargetPosition(newTarget);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            runtime.reset();
            motor.setPower(0.1);
            while (opModeIsActive() && motor.isBusy()) {
                dataflow.sendToAll(new String[]{"Running to", "Currently at", "Velocity", "Current"},
                                                newTarget, motor.getCurrentPosition(), motor.getVelocity(), motor.getCurrent(CurrentUnit.MILLIAMPS));
            }

            // Stop the motor and reset the mode
            motor.setPower(0);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sleep(2500);  // Pause for a while before ending the test
        }
    }
}
