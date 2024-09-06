package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.BASE.*;
import static org.firstinspires.ftc.teamcode.Constants.FIELD.*;
import static org.firstinspires.ftc.teamcode.Constants.SWIVEL.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Dataflow;

public class AutoSystems extends Drivebase {
    private final Telemetry telemetry; //create obj for telemetry, IMU and LinearOpMode
    private final IMUHandler imuHandler;
    private final LinearOpMode linearOpMode;
    private Dataflow dataflow;

    public AutoSystems(LinearOpMode linearOpMode) { //create an access for LinearOpModes and initialize the IMU
        super(linearOpMode);
        this.linearOpMode = linearOpMode;
        this.telemetry = linearOpMode.telemetry;
        this.imuHandler = new IMUHandler(linearOpMode);
        this.imuHandler.init();
        this.imuHandler.resetHeading();
        this.dataflow = new Dataflow(this.telemetry);
    }

    public void turnToHeading(double heading) { //return telemetry while the robot is in auto mode and turning
        while (linearOpMode.opModeIsActive() && !onHeading(TURN_SPEED, heading, P_TURN_GAIN)) { 
            telemetry.update(); //telemetry transmission while the robot is turning
        }
        setMotorsPower(0, 0); //stop the motor after turn to the desired value
    }

    private double whichWayToTurn(double targetHeading) { //return the value for robot to rotate clockwise or counter-clockwise
        double currentHeading = imuHandler.getHeading();
        if (currentHeading >= -targetHeading && currentHeading <= targetHeading) {
            return targetHeading - abs(currentHeading);
        }
        return abs(currentHeading) - targetHeading;
    }

    public void holdHeading(double heading, double holdTime) { //run continuously the onHeading function to get to the desired heading
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (holdTimer.time() < holdTime) { //if the robot has got to the desired heading value, stop the motor
            onHeading(TURN_SPEED, heading, P_TURN_GAIN);
        }
        setMotorsPower(0, 0); 
    }

    private boolean onHeading(double speed, double heading, double PCoeff) {
        ElapsedTime runtime = new ElapsedTime(); //track runtime
        runtime.reset();
        double error = getError(heading);
        double steer;
        boolean onTarget = false; 
        double leftSpeed; //declare variables for motors value
        double rightSpeed;

        if (abs(error) <= HEADING_THRESHOLD) { //if the error value are lower than the heading threshold, stop the motor
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else { //if else, calculate the motor power based on the value returned from the PCoeff 
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        setMotorsPower(leftSpeed, rightSpeed); //set motors power
        dataflow.addToAll(new String[] {"Target", "Err", "Current heading", "Speed left", "Speed right", "Runtime"},
                                        heading, error, imuHandler.getHeading(), leftSpeed, rightSpeed, runtime); 
        dataflow.sendDatas(); //telemetry transmision
        return onTarget;
    }

    private double getError(double targetAngle) { //return the heading_error to use in P control of turning
        double robotError = targetAngle - imuHandler.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) { //return the steer value of P Control in turning
        return Range.clip(error * PCoeff, -1, 1);
    }

    private void setUpForEncoder(double moveTarget, DcMotorEx motorEx, double COUNTS_PER_INCH) { //make the motor to go to the desired encoder value
        int newTarget = motorEx.getCurrentPosition() + (int) (moveTarget * COUNTS_PER_INCH); //calculate the encoder ticks from real distance to set new target
        motorEx.setTargetPosition(newTarget);
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set the motor to go to the target
    }

    public void horizontalMove(double moveTarget) { //middle motor to go to the desired encoder value
        setUpForEncoder(moveTarget, middleWheel, HD_SMALL_COUNTS_PER_INCH); //run to the desired position
        middleWheel.setPower(AUTO_DRIVE);
        while (linearOpMode.opModeIsActive() && middleWheel.isBusy()) {
            // Wait for the movement to complete
        }
        middleWheel.setPower(0);
        middleWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
