// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimbPosition;
import frc.robot.subsystems.Climber.PIDProfile;
import frc.robot.util.PIDConfiguration;

import java.util.HashMap;
import java.util.Map;

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
        public static final int TALON_UNITS_PER_REV = 2048; // TalonFX integrated encoder
        public static final double GEARING = 10.71;
        public static final double WHEEL_DIAMETER_IN = 6;
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(WHEEL_DIAMETER_IN);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_M * Math.PI;
        public static final double TRACK_WIDTH_M = Units.inchesToMeters(21.869); // From CAD
        public static final SimpleMotorFeedforward MOTOR_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0); // From Sysid
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_M);
        public static final double MAX_BATTERY_V = 12.0;
        public static final boolean HAS_ENCODERS = true; // Set to false if encoders are not functioning
    }

    public static class Arm {
        public static final int MOTOR = 5;
        public static final int TALON_UNITS_PER_REV = 2048; // TalonFX integrated encoder
        public static final int GEARING = 48 * 5; // 48 (4:3:3) from MAXPlanetary gearbox, 5 (12:60) from sprockets
        // Arm Setpoint
        public static final double DESIRED_DEGREES = 81; // 68.37 in robot CAD
        public static final double DESIRED_ROTATIONS = DESIRED_DEGREES / 360;
        // Position PID Gains
        public static final double UP_PID_P = 0.04;
        public static final double UP_PID_I = 0;
        public static final double UP_PID_D = 0;
        public static final double DOWN_PID_P = UP_PID_P;
        public static final double DOWN_PID_I = 0;
        public static final double DOWN_PID_D = 0;
        // Motion profile constraint
        public static final TrapezoidProfile.Constraints TRAPEZOID_CONSTRAINTS =
                new TrapezoidProfile.Constraints(5.5, 5); // rad/s and rad/s^2
    }

    public static class BallManipulator {
        public static final int MOTOR = 6;
    }

    public static class Climber {
        public static final int LEFT_MOTOR = 6;
        public static final int RIGHT_MOTOR = 7;
        public static final int LEFT_SOLENOID_FORWARD = 0;
        public static final int LEFT_SOLENOID_REVERSE = 1;
        public static final int RIGHT_SOLENOID_FORWARD = 2;
        public static final int RIGHT_SOLENOID_REVERSE = 3;
        public static final int TALON_UNITS_PER_REV = 2048; // TalonFX integrated encoder
        public static final int GEARING = 36; // 36 from MAXPlanetary gearbox
        public static final double WINCH_DIAMETER_M = Units.inchesToMeters(0.5);
        // Position PID Gains
        public static final Map<PIDProfile, PIDConfiguration> PID_CONSTANTS = Map.of(
            PIDProfile.NO_LOAD_UP,   PIDConfiguration.of(0, 0, 0),
            PIDProfile.NO_LOAD_DOWN, PIDConfiguration.of(0, 0, 0),
            PIDProfile.LOADED_UP,    PIDConfiguration.of(0, 0, 0),
            PIDProfile.LOADED_DOWN,  PIDConfiguration.of(0, 0, 0)
        );
    }
}
