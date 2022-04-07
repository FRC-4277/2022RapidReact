// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDConfiguration;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private static final double MANUAL_NORMAL_SPEED = 0.4;
  private static final double MANUAL_FAST_SPEED = 0.75;
  private static final boolean USE_FOLLOW_MODE = false;
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

  // Shuffleboard
  private final ShuffleboardTab tab = Shuffleboard.getTab("Climber");
  private final ShuffleboardTab mainTab;
  private final NetworkTableEntry leftPositionEntry, rightPositionEntry;
  private final NetworkTableEntry manualOverrideEntry;

  /** Creates a new Climber.
   * @param mainTab*/
  public Climber(ShuffleboardTab mainTab) {
    this.mainTab = mainTab;
    motors.forEach(this::configureMotor);
    leftMotor.setInverted(LEFT_MOTOR_INVERSION);
    rightMotor.setInverted(RIGHT_MOTOR_INVERSION);
    // Soft limit up
    leftMotor.configForwardSoftLimitThreshold(metersToSensorUnits(ClimbPosition.MAXIMUM.getHeightMeters()
            + Units.inchesToMeters(LEFT_ADDITIONAL_OFFSET_IN)));
    leftMotor.configForwardSoftLimitEnable(true);
    rightMotor.configForwardSoftLimitThreshold(metersToSensorUnits(ClimbPosition.MAXIMUM.getHeightMeters()
            + Units.inchesToMeters(RIGHT_ADDITIONAL_OFFSET_IN)));
    rightMotor.configForwardSoftLimitEnable(true);

    if (USE_FOLLOW_MODE) {
      rightMotor.follow(leftMotor);
    }

    leftPositionEntry = tab.add("Left Position", "")
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    rightPositionEntry = tab.add("Right Position", "")
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();

    manualOverrideEntry = mainTab.add("Climber Manual Override", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(5, 1)
            .withSize(2, 1).getEntry();
    manualOverrideEntry.addListener(notification -> setManualOverride(notification.value.getBoolean()),
            EntryListenerFlags.kUpdate);
  }

  private void setManualOverride(boolean override) {
    motors.forEach(motor -> {
      motor.configForwardSoftLimitEnable(!override);
      motor.configReverseSoftLimitEnable(!override);
    });
  }

  public void configureMotor(TalonFX motor) {
    // Reset motor & configure brake mode
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    // Setup encoder
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100); // PID index 0, 100ms timeout
    motor.setSelectedSensorPosition(0);
    // Setup soft limit down
    motor.configReverseSoftLimitThreshold(0);
    motor.configReverseSoftLimitEnable(true);
    // Setup position PID gains
    configurePID(PIDProfile.NO_LOAD_UP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftPositionEntry.setString(getPositionEntry(leftMotor));
    rightPositionEntry.setString(getPositionEntry(rightMotor));
  }

  private String getPositionEntry(TalonFX motor) {
    int units = (int) motor.getSelectedSensorPosition();
    double meters = sensorUnitsToMeters(units);
    return String.format("%d u / %.2f m / %.2f in",
            units, meters, Units.metersToInches(meters));
  }

  public void moveManual(double speed) {
    if (USE_FOLLOW_MODE) {
      leftMotor.set(TalonFXControlMode.PercentOutput, speed);
    } else {
      motors.forEach(motor -> motor.set(TalonFXControlMode.PercentOutput, speed));
    }
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

  public void holdPosition(ClimbPosition position, boolean loaded, boolean up) {
    configurePID(PIDProfile.fromSettings(loaded, up));
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

  public void moveManual(ClimberDirection direction, boolean fast) {
    moveManual((direction == ClimberDirection.UP ? 1 : -1) * (fast ? MANUAL_FAST_SPEED : MANUAL_NORMAL_SPEED));
  }

  public enum ClimbPosition {
    HOME(0),
    MAXIMUM(Units.inchesToMeters(12.0));

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

    private static final Map<Map.Entry<Boolean, Boolean>, PIDProfile> profileMap = new HashMap<>();

    private final boolean loaded;
    private final boolean up;

    PIDProfile(boolean loaded, boolean up) {
      this.loaded = loaded;
      this.up = up;
    }

    public boolean isLoaded() {
      return loaded;
    }

    public boolean isUp() {
      return up;
    }

    public static PIDProfile fromSettings(boolean loaded, boolean up) {
      if (profileMap.size() == 0) {
        for (PIDProfile profile : values()) {
          profileMap.put(Map.entry(loaded, up), profile);
        }
      }
      return profileMap.get(Map.entry(loaded, up));
    }
  }

  public enum ClimberDirection {
    UP, DOWN
  }
}
