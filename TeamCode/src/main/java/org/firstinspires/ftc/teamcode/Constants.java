package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static class BASE {
        public static final double COUNTS_PER_HD_MOTOR_REV = 28 ;
        public static final double COUNTS_PER_HEX_MOTOR_REV = 288.0;
        public static final double DRIVE_GEAR_REDUCTION = 16.0;
        public static final double WHEEL_DIAMETER_INCHES = 9.00/2.54;
        public static final double SMALL_WHEEL_DIAMETER_INCHES = 6.00/2.54;
        public static final double CORE_HEX_RPM = 125;
        public static final double DC_HEX_RPM = 300;
        public static final double HD_COUNTS_PER_INCH = (COUNTS_PER_HD_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    }

    public static class SPEED {
        public static final double BOOST_DRIVE = 0.4;
        public static final double NORMAL_DRIVE = 0.2;
        public static final double LINEAR_SPEED = 1.0;
    }

    public static class SENSE {
        public static final double JOYSTICK_SENSE = 0.05;
        public static final float TRIGGER_SENSE = 0.2f;
    }

    public static final class ROTATION {
        public static final double ROTATE_KP = 1.3;
        public static final double ROTATE_KI = 0;
        public static final double ROTATE_KD = 0;
        public static final double TOLERANCE = 0.05;
        public static final double INTERGRAL_MIN = 0;
        public static final double INTERGRAL_MAX = 1;
    }
}
