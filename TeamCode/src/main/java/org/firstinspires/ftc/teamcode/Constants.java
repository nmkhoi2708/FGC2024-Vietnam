package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    @Config
    public static class BASE {
        public static double COUNTS_PER_HD_MOTOR_REV = 28;
        public static double COUNTS_PER_HEX_MOTOR_REV = 288.0;
        public static double DRIVE_GEAR_REDUCTION = 13.0321;
        public static double WHEEL_DIAMETER_INCHES = 9.00 / 2.54;
        public static double SMALL_WHEEL_DIAMETER_INCHES = 6.00 / 2.54;
        public static double CORE_HEX_RPM = 125;
        public static double DC_HEX_RPM = 300;

        // These values are computed based on the above constants.
        public static double HD_COUNTS_PER_INCH = (COUNTS_PER_HD_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static double HD_SMALL_COUNTS_PER_INCH = (COUNTS_PER_HD_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (SMALL_WHEEL_DIAMETER_INCHES * Math.PI);
        public static double HEX_COUNT_PER_INCH = (COUNTS_PER_HEX_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);
    }

    @Config
    public static class SPEED {
        public static double BOOST_DRIVE = 0.4;
        public static double NORMAL_DRIVE = 0.2;
        public static double LINEAR_SPEED = 1.0;
        public static double AUTO_DRIVE = 0.2;
    }

    @Config
    public static class SENSE {
        public static double JOYSTICK_SENSE = 0.05;
        public static float TRIGGER_SENSE = 0.2f;
    }

    @Config
    public static class SWIVEL {
        public static double TURN_SPEED = 0.3;
        public static double HEADING_THRESHOLD = 0.2;
        public static double P_TURN_GAIN = 0.02;
    }

    @Config
    public static class FIELD {
        // Taken from Official field OnShape CAD
        public static double HORIZONTAL_AUTO_REQUIREMENT = 30.03706;
    }
}
