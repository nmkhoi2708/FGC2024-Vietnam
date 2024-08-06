package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RampUpMotorPower", group="TeleOp")
@Disabled
public class RampUpMotorPower extends OpMode {

    private DcMotor motor;
    private double motorPower = 0.0;
    private double increment = 0.6; // Change this value to control the speed of the ramp-up
    private double maxPower = 1.0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setPower(0);
    }

    @Override
    public void loop() {
        double joystickValue = gamepad1.left_stick_y; // Assuming we're using the left stick y-axis for this example

        if (joystickValue == 0) {
            motorPower = 0;
        } else {
            if (motorPower < maxPower) {
                motorPower += increment;
                if (motorPower > maxPower) {
                    motorPower = maxPower;
                }
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        motor.setPower(motorPower);

        telemetry.addData("Joystick Value", joystickValue);
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
