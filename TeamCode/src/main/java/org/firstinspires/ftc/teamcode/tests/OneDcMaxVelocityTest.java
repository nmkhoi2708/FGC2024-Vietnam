package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Dataflow;

@TeleOp
@Disabled
public class OneDcMaxVelocityTest extends LinearOpMode {
    Dataflow dataflow;
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "DcHex");
        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = motor.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            dataflow.sendToAll(new String[]{"Current velocity", "Maximum velocity"}, currentVelocity ,maxVelocity);
        }
    }
}
