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

        if (joystickValue == 0) { //stop the motor when the joystick is released
            motorPower = 0;
        } else {
            if (motorPower < maxPower) {
                motorPower += increment;  //increase the motor power gradually ( plus motorPower each loop until it reaches maxpower )
                if (motorPower > maxPower) {
                    motorPower = maxPower;
                }
                try {
                    Thread.sleep(1000); //pause the execution of the current thread
                } catch (InterruptedException e) {
                    e.printStackTrace(); //provide information about the thread
                }
            }
        }

        motor.setPower(motorPower); //set motor power

        telemetry.addData("Joystick Value", joystickValue); //telemetry transmission
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
