// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    }

    public static class Arm {
        public static final int MOTOR = 5;
        public static final int TALON_UNITS_PER_REV = 2048;
        public static final int GEARING = 16 * 5; // 16 from VersaPlanetary gearbox, 5 from sprockets
        // Motion Magic
        public static final int CRUISE_VELOCITY_RPM = 6380 / 8; // eighth free speed
        public static final int CRUISE_VELOCITY_NATIVE = CRUISE_VELOCITY_RPM * 60 * TALON_UNITS_PER_REV / 10; // converting RPM to units/100ms
        public static final int CRUISE_ACCELERATION_NATIVE = (int) Math.round(CRUISE_VELOCITY_NATIVE / 0.5); // "0.5" secs to reach cruise velocity
        public static final int S_CURVE_SMOOTHING = 5;
        public static final double DESIRED_DEGREES = 60; // 68.37 in robot CAD
        public static final double DESIRED_ROTATIONS = DESIRED_DEGREES / 360;
        // Position PID
        public static final double PID_P = 0.01;
        public static final double PID_I = 0;
        public static final double PID_D = 0;
        // PID Feedforward
        public static final TrapezoidProfile.Constraints TRAPEZOID_CONSTRAINTS = new TrapezoidProfile.Constraints(0.25, 0.5); // rad/s and rad/s^2
        public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0, 0, 0); // Feedforward TODO

    }

    public static class BallManipulator {
        public static final int MOTOR = 6;
    }
    
}
