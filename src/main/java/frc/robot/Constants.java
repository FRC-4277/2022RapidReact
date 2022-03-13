// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class Joysticks {
        // USB Ports
        public static final int LOGITECH_JOYSTICK = 0;
        public static final int XBOX_CONTROLLER = 1;
    }
    public static class DriveTrain {
        public static final int FRONT_LEFT = 1;
        public static final int FRONT_RIGHT = 2;
        public static final int BACK_LEFT = 3;
        public static final int BACK_RIGHT= 4;
        // Autonomous Driving
        public static final double WHEEL_DIAMETER_IN = 6;
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(WHEEL_DIAMETER_IN);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_M * Math.PI;
        public static final int TALON_UNITS_PER_REV = 2048;
        public static final double GEARING = 10.71;
    }

    public static class Arm {
        public static final int MOTOR = 5;
        public static final int TALON_UNITS_PER_REV = 2048;
        public static final int GEARING = 3 * 16 * 5; // 48 from VersaPlanetary gearbox, 5 from sprockets
        // Motion Magic
        //public static final int CRUISE_VELOCITY_RPM = 6380 / 8; // eighth free speed
        //public static final int CRUISE_VELOCITY_NATIVE = CRUISE_VELOCITY_RPM * 60 * TALON_UNITS_PER_REV / 10; // converting RPM to units/100ms
        //public static final int CRUISE_ACCELERATION_NATIVE = (int) Math.round(CRUISE_VELOCITY_NATIVE / 0.5); // "0.5" secs to reach cruise velocity
        //public static final int S_CURVE_SMOOTHING = 5;
        public static final double DESIRED_DEGREES = 81; // 68.37 in robot CAD
        public static final double DESIRED_ROTATIONS = DESIRED_DEGREES / 360;
        // Position PID
        public static final double UP_PID_P = 0.04;
        public static final double UP_PID_I = 0;
        public static final double UP_PID_D = 0;
        public static final double DOWN_PID_P = 0.04;
        public static final double DOWN_PID_I = 0;
        public static final double DOWN_PID_D = 0;
        // PID Feedforward
        public static final TrapezoidProfile.Constraints TRAPEZOID_CONSTRAINTS = new TrapezoidProfile.Constraints(5.5, 5); // rad/s and rad/s^2
        public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0, 0, 0); // Feedforward TODO

    }

    public static class BallManipulator {
        public static final int MOTOR = 6;
    }
    
}
