package org.firstinspires.ftc.teamcode.utils;

public class WheelOdometry {
    private final double WHEEL_RADIUS = 3.54; // WHEEL_RADIUS
    private final double TICKS_PER_REV = 538; // Manually Calibrated
    private final double TRACK_WIDTH = 14.5;
    private double x, y, theta;
    private double prevL = 0, prevR = 0; // previously calculated vals
    private String outStr;

    public WheelOdometry(double x, double y, double theta){ //create a wheel odo class
        this.x = x;
        this.y  = y;
        this.theta = theta;
    }

    public void updatePositionWithIMU(double currLTick, double currRTick, double currTheta){
        double dL =  (2*Math.PI*WHEEL_RADIUS) * ((currLTick - prevL) / TICKS_PER_REV); //calculate the change in left and right motor encoder
        double dR =  (2*Math.PI*WHEEL_RADIUS) * ((currRTick - prevR) / TICKS_PER_REV);
        double dC = (dL + dR) / 2; //the change at the middle wheel

        double dX = dC * Math.cos(theta); //calculate the change in x and y axis
        double dY = dC * Math.sin(theta);
        x += dX; y += dY; theta = Math.toRadians(currTheta); //update the current pose
        outStr = "xPos: " + format(x) + "\nyPos: " + format(y) + "\nAngle: " + format(theta);//telemetry transmission

        prevL = currLTick; //save the prev loop value to use for later loop
        prevR = currRTick;
    }
    // Apply the iterative process
    public void updatePosition(double currLTick, double currRTick){ //update the odometry pose without using the IMU
        double dL =  (2*Math.PI*WHEEL_RADIUS) * ((currLTick - prevL) / TICKS_PER_REV);
        double dR =  (2*Math.PI*WHEEL_RADIUS) * ((currRTick - prevR) / TICKS_PER_REV);
        double dC = (dL + dR) / 2;

        double dX = dC * Math.cos(theta);
        double dY = dC * Math.sin(theta);
        x += dX;
        y += dY;
        theta = theta + ((dR - dL)/TRACK_WIDTH); //calculate the angle of the robot

        outStr = "xPos: " + format(x) + "\nyPos: " + format(y) + "\nAngle: " + format(theta);

        prevL = currLTick;
        prevR = currRTick;
    }


    // reset pose
    public void setPose(int x, int y, int theta){
        this.x = x;
        this.y = y ;
        this.theta = theta;
    }

    // Output positions to telemetry
    public String displayPositions(){
        return outStr;
    }

    private String format(double num){
        return String.format("%.3f", num);
    }
}