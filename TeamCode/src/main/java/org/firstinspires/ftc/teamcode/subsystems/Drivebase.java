package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;


public class Drivebase {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx middleWheel;
    private final HardwareMap hardwareMap;
    private double speed = NORMAL_DRIVE;
    private IMU imu;
    private List<DcMotorEx> motors, leftMotors, rightMotors;


    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    public Drivebase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        middleWheel = hardwareMap.get(DcMotorEx.class, "middleHex");

        motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront);
        rightMotors = Arrays.asList(rightFront, rightBack);
        leftMotors = Arrays.asList(leftFront, leftBack);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDirection(DcMotorSimple.Direction.REVERSE, rightMotors);
    }

    public void setMotorsPower(double leftPower, double rightPower){
        leftFront.setPower(leftPower * speed);
        leftBack.setPower(leftPower * speed);
        rightFront.setPower(rightPower * speed);
        rightBack.setPower(rightPower * speed);
    }

    public void setHorizontalMove(double hex_power) {
        double power;
        if(abs(hex_power) >= 1.0) {
            power = Range.clip(hex_power, -1.0, 1.0);
        } else {
            power = 0;
        }
        middleWheel.setPower(power);
    }

    public int getPosition(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void boost() {
        if(speed == NORMAL_DRIVE) {
            speed = BOOST_DRIVE;
        } else {
            speed = NORMAL_DRIVE;
        }
    }

    public double getPower(DcMotorEx motorEx) {
        return motorEx.getPower();
    }

    public double getLeftPower() {
        return getPower(leftBack);
    }

    public double getRightPower() {
        return getPower(rightBack);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setDirection(DcMotorSimple.Direction Direction, List<DcMotorEx> sideMotors) {
        for (DcMotorEx motor : sideMotors) {
            motor.setDirection(Direction);
        }
    }

    public void oneMotorByEncoder(double moveTarget,
                                  double power,
                                  DcMotorEx motorEx,
                                  double COUNTS_PER_INCH) {
        int newTarget = motorEx.getCurrentPosition() + (int) (moveTarget*COUNTS_PER_INCH);
        motorEx.setTargetPosition(newTarget);
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorEx.setPower(power);
        motorEx.setPower(0);
        motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveBaseByEncoder(double rightTarget,
                                   double leftTarget,
                                   double COUNT_PER_INCH,
                                   DcMotorEx[] leftMotors,
                                   DcMotorEx[] rightMotors) {
        int newLeftTarget;
        int newRightTarget;

        for(DcMotorEx motor : leftMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void autoHorizontalMove() {
        double length = 30.03706;
    }

    public void turnToHeading(double speed, double heading) {
        while (!onHeading(speed, heading, P_TURN_GAIN)) {
            // Telemetry update
        }
    }

    public void holdHeading(double speed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {
            onHeading(speed, heading, P_TURN_GAIN);
        }
        setMotorsPower(0, 0);  // Stop all motion
    }

    private boolean onHeading(double speed, double heading, double PCoeff) {
        double error = getError(heading);
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        setMotorsPower(leftSpeed, rightSpeed);

        return onTarget;
    }

    private double getError(double targetAngle) {
        double robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
