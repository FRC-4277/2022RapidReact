// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDConfiguration;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(LEFT_MOTOR);
  private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR);
  private final List<TalonFX> motors = List.of(leftMotor, rightMotor);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
          LEFT_SOLENOID_FORWARD, LEFT_SOLENOID_REVERSE);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
          RIGHT_SOLENOID_FORWARD, RIGHT_SOLENOID_REVERSE);
  private final List<DoubleSolenoid> solenoids = List.of(leftSolenoid, rightSolenoid);

  // State variables
  private PIDProfile pidConfiguredFor;

  /** Creates a new Climber. */
  public Climber() {
    motors.forEach(this::configureMotor);
  }

  public void configureMotor(TalonFX motor) {
    // Reset motor & configure brake mode
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    // Setup encoder
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100); // PID index 0, 100ms timeout
    motor.setSelectedSensorPosition(0);
    // Setup soft limit both ways
    motor.configForwardSoftLimitThreshold(metersToSensorUnits(ClimbPosition.MAXIMUM.getHeightMeters()));
    motor.configForwardSoftLimitEnable(true);
    motor.configReverseSoftLimitThreshold(0);
    motor.configReverseSoftLimitEnable(true);
    // Setup position PID gains
    configurePID(PIDProfile.NO_LOAD_UP);
  }

  public void moveManual(double speed) {
    motors.forEach(motor -> motor.set(TalonFXControlMode.PercentOutput, speed));
  }

  public void stop(){
    moveManual(0);
  }

  /**
   * Sets target position to that of a trapezoid profile state
   * @param state Trapezoid profile state in m and m/s
   */
  public void moveToState(TrapezoidProfile.State state) {
    holdPositionMeters(state.position);
  }

  public void holdPositionMeters(double meters) {
    motors.forEach(motor -> motor.set(TalonFXControlMode.Position, metersToSensorUnits(meters)));
  }

  public void holdPosition(ClimbPosition position, boolean loaded) {
    holdPositionMeters(position.getHeightMeters());
  }

  public void setSolenoids(DoubleSolenoid.Value value) {
    solenoids.forEach(solenoid -> solenoid.set(value));
  }

  public double sensorUnitsToRotations(double sensorUnits) {
    return (sensorUnits / TALON_UNITS_PER_REV) / GEARING;
  }

  public double rotationsToSensorUnits(double rotations) {
    return rotations * GEARING * TALON_UNITS_PER_REV;
  }

  public double sensorUnitsToMeters(double sensorUnits) {
    return sensorUnitsToRotations(sensorUnits) * (Math.PI * WINCH_DIAMETER_M);
  }

  public double metersToSensorUnits(double meters) {
    return rotationsToSensorUnits(meters / (Math.PI * WINCH_DIAMETER_M));
  }

  public void configurePID(PIDProfile profile) {
    if (pidConfiguredFor != profile) {
      configurePID(PID_CONSTANTS.get(profile));
      pidConfiguredFor = profile;
    }
  }

  private void configurePID(PIDConfiguration configuration) {
    motors.forEach(motor -> motor.config_kP(0, configuration.getkP()));
    motors.forEach(motor -> motor.config_kI(0, configuration.getkI()));
    motors.forEach(motor -> motor.config_kD(0, configuration.getkD()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ClimbPosition {
    HOME(0),
    MAXIMUM(Units.inchesToMeters(40));

    private final double heightMeters;

    ClimbPosition(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public double getHeightMeters() {
      return heightMeters;
    }
  }

  public enum PIDProfile {
    NO_LOAD_UP(false, true),
    NO_LOAD_DOWN(false, false),
    LOADED_UP(true, true),
    LOADED_DOWN(true, false);

    private static final Map<Integer, PIDProfile> lookupMap = new HashMap<>();
    private final boolean loaded;
    private final boolean up;
    private int flags = 0;

    PIDProfile(boolean loaded, boolean up) {
      this.loaded = loaded;
      this.up = up;
      if (loaded) {
        flags |= 1;
      }
      if (up) {
        flags |= 2;
      }
    }

    public boolean isLoaded() {
      return loaded;
    }

    public boolean isUp() {
      return up;
    }

    public int getFlags() {
      return flags;
    }

    public static PIDProfile from(boolean loaded, boolean up) {
      if (lookupMap.size() == 0) {
        for (PIDProfile profile : values()) {
          lookupMap.put(profile.getFlags(), profile);
        }
      }

      int flags = 0;
      if (loaded) {
        flags |= 1;
      }
      if (up) {
        flags |= 2;
      }
      return lookupMap.get(flags);
    }
  }
}
