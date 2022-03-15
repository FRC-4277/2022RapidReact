// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
  public static final double SPEED = 0.35; // Speed to move the arm manually
  public static final double SLOW_SPEED = 0.25; // Speed to move "slowly"
  public static final double SLOW_ZONE_DEGREES = 10; // Slow down within <x> degrees of end, when going down
  public static final double DEGREES_THRESHOLD = 5.0; // Degrees away from a position to be considered at the position
  public static final double LIMIT_SWITCH_ZONE_DEGREES = 15; // Degrees of zone to allow limit switch to zero

  private final TalonFX motor = new TalonFX(MOTOR);

  // Shuffleboard
  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  private NetworkTableEntry armPositionEntry, hasBeenZeroedEntry, limitSwitchSafetySetting;

  // State Variables
  private ArmDirection pidDirection = null; // Direction that PID is currently configured for
  private boolean hasBeenZeroed = false; // Whether the limit switch has been hit yet
  private boolean isLimitSwitchResetEnabled;

  /** Creates a new Arm. */
  public Arm() {
    // Reset motor config, configure inversion, and set to brake mode
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);

    // Select & reset encoder to up position
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100); // PID index 0, 100ms timeout
    motor.setSelectedSensorPosition(rotationsToSensorUnits(DESIRED_ROTATIONS));

    // Configure reverse (down) limit switch & have it reset encoder
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    setLimitSwitchEnabled(true);
    isLimitSwitchResetEnabled = true;

    // Enable Soft Limit to ensure arm doesn't go too far
    motor.configForwardSoftLimitThreshold(rotationsToSensorUnits(DESIRED_ROTATIONS));
    motor.configForwardSoftLimitEnable(true);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    armPositionEntry = tab.add("Position", "")
            .withPosition(0, 0)
            .withSize(3, 1)
            .getEntry();
    hasBeenZeroedEntry = tab.add("Has Been Zeroed", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();
    limitSwitchSafetySetting = tab.add("Enable Limit Switch Safe Zone", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    // Default to enabling limit switch in case setting changes; periodic will override with correct value
    limitSwitchSafetySetting.addListener(notification -> setLimitSwitchEnabled(true),
            EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);
  }

  @Override
  public void periodic() {
    // Set zeroed variable to true if the limit switch is ever hit
    if (!hasBeenZeroed && (motor.isRevLimitSwitchClosed() == 1)) {
      hasBeenZeroed = true;
    }

    // Special logic to make the limit switch not zero the arm UNLESS arm is near the bottom
    // Only perform this logic if the bot has already zeroed its arm at least once
    if (hasBeenZeroed) {
      /* Have limit switch state be true if either are true:
      - Limit switch safety setting is FALSE
      - Limit switch safety setting is TRUE, and we're in the switch zone
       */

      boolean safetyEnabled = limitSwitchSafetySetting.getBoolean(true);
      boolean desiredLimitSwitchState = !safetyEnabled || getPositionDegrees() <= LIMIT_SWITCH_ZONE_DEGREES;
      // Update TalonFX limit switch setting only if it's changed
      if (isLimitSwitchResetEnabled != desiredLimitSwitchState) {
        setLimitSwitchEnabled(desiredLimitSwitchState);
        isLimitSwitchResetEnabled = desiredLimitSwitchState;
      }
    }

    // Shuffleboard
    armPositionEntry.setString(String.format("%d u / %.2f deg / %.2f rad",
            (int) motor.getSelectedSensorPosition(), getPositionDegrees(), Math.toRadians(getPositionDegrees())));
    hasBeenZeroedEntry.setBoolean(hasBeenZeroed);
  }

  private void setLimitSwitchEnabled(boolean enabled) {
    motor.configClearPositionOnLimitR(enabled, 100); //100 ms timeout
  }

  public void moveManualSlow(ArmDirection direction) {
    motor.set(ControlMode.PercentOutput, (direction == ArmDirection.UP ? 1 : -1) * SLOW_SPEED);
  }

  public void moveManual(ArmDirection direction) {
    motor.set(ControlMode.PercentOutput, (direction == ArmDirection.UP ? 1 : -1) * SPEED);
  }

  public void stopMoving() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Sets target position to that of a trapezoid profile state
   * @param state Trapezoid profile state in rad and rad/s
   */
  public void moveToState(TrapezoidProfile.State state) {
    holdPositionRad(state.position);
  }

  public void holdPosition(ArmPosition position) {
    // Special case for holding down is just to keep the motor running downwards, as it will hit the limit switch
    if (position == ArmPosition.DOWN) {
      moveManualSlow(ArmDirection.DOWN);
      return;
    }
    // Otherwise, for all other positions, use PID to hold
    holdPositionRad(Math.toRadians(position.getDegrees()));
  }

  public void holdPositionRad(double targetAngleRad) {
    // Configure PID direction depending on if the target position is above or below
    configurePID(getPositionRad() < targetAngleRad ? ArmDirection.UP : ArmDirection.DOWN);
    // Use TalonFX position PID
    motor.set(TalonFXControlMode.Position, radToSensorUnits(targetAngleRad));
  }

  public boolean isAtPosition(ArmPosition position) {
    return Math.abs(getPositionDegrees() - position.getDegrees()) <= DEGREES_THRESHOLD;
  }

  public double getPositionRotations() {
    return sensorUnitsToRotations(motor.getSelectedSensorPosition());
  }

  public double getPositionDegrees() {
    return getPositionRotations() * 360;
  }

  public double getPositionRad() {
    return getPositionRotations() * Math.PI * 2;
  }
  public void configurePID(ArmDirection direction) {
    SmartDashboard.putString("Arm PID Dir", direction.toString());
    if (pidDirection != direction) {
      motor.config_kP(0, direction == ArmDirection.UP ? UP_PID_P : DOWN_PID_P);
      motor.config_kI(0, direction == ArmDirection.UP ? UP_PID_I : DOWN_PID_I);
      motor.config_kD(0, direction == ArmDirection.UP ? UP_PID_D : DOWN_PID_D);
      pidDirection = direction;
    }
  }

  public double sensorUnitsToRotations(double sensorUnits) {
    return (sensorUnits / TALON_UNITS_PER_REV) / GEARING;
  }

  public int degreesToSensorUnits(double degrees) {
    return rotationsToSensorUnits(degrees / 360);
  }

  public int radToSensorUnits(double rad) {
    return rotationsToSensorUnits(rad / (Math.PI * 2));
  }

  public int rotationsToSensorUnits(double rotations) {
    return (int) Math.round(rotations * GEARING * TALON_UNITS_PER_REV);
  }

  public TrapezoidProfile.State getTrapezoidState() {
    return new TrapezoidProfile.State(getPositionRad(), 0);
  }

  public enum ArmDirection {
    UP, DOWN
  }

  public enum ArmPosition {
    DOWN(0),
    CLIMB(40),
    UP(DESIRED_DEGREES);

    private final double degrees;

    ArmPosition(double degrees) {
      this.degrees = degrees;
    }

    public double getDegrees() {
      return degrees;
    }
  }
}
