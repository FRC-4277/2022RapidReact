// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
  public static final double SPEED = 0.2;
  public static final double SLOW_ZONE_DEGREES = 10; // Slow down within 10 degrees of end
  public static final double SLOW_SPEED = 0.2;

  private final TalonFX motor = new TalonFX(MOTOR);
  private ArmDirection pidDirection = null; // Direction that PID is currently configured for
  private boolean hasBeenZeroed = false; // Whether the limit switch has been hit yet

  /** Creates a new Arm. */
  public Arm() {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100); // PID index 0, 100ms timeout

    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor.configClearPositionOnLimitR(true, 100); //100 ms timeout

    // Enable Soft Limit to ensure arm doesn't go too far
    motor.configForwardSoftLimitThreshold(rotationsToSensorUnits(DESIRED_ROTATIONS));
    motor.configForwardSoftLimitEnable(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Native", motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Degrees", getPositionDegrees());
    SmartDashboard.putNumber("Arm Rad", Math.toRadians(getPositionDegrees()));
    // Set zeroed variable to true if the limit switch is ever hit
    if (!hasBeenZeroed && (motor.isRevLimitSwitchClosed() == 1)) {
      hasBeenZeroed = true;
    }
    SmartDashboard.putBoolean("Has Been Zeroed", hasBeenZeroed);
  }


  public void moveDownManualSlow() {
    motor.set(ControlMode.PercentOutput, -SLOW_SPEED);
  }


  public void moveUp(double speed) {
    motor.set(ControlMode.PercentOutput, Math.abs(speed));
  }

  public void moveUpManualSlow() {
    motor.set(ControlMode.PercentOutput, SLOW_SPEED);
  }

  public void moveManual(ArmDirection direction) {
    motor.set(ControlMode.PercentOutput, (direction == ArmDirection.UP ? 1 : -1) * SPEED);
  }

  public void stopMoving() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void moveDownAutomatic(TrapezoidProfile.State state) {
    if (getPositionDegrees() <= 10) {
      // Position PID to hold zero
      if (hasBeenZeroed && state != null) {
        configurePID(ArmDirection.DOWN);
        holdPosition(0, false);
      } else {
        moveDownManualSlow();
      }
    } else {
      if (hasBeenZeroed && state != null) {
        configurePID(ArmDirection.DOWN);
        holdPosition(state.position, false);
      } else {
        moveManual(ArmDirection.DOWN);
      }
    }
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

  public void moveUpAutomatic(TrapezoidProfile.State state) {
    configurePID(ArmDirection.UP);
    holdPosition(state.position, false);
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

  public void holdPosition(double positionRad, boolean setPID) {
    if (setPID) {
      if (getPositionRad() < positionRad) {
        configurePID(ArmDirection.UP);
      } else {
        configurePID(ArmDirection.DOWN);
      }
    }
    double targetUnits = radToSensorUnits(positionRad);
    motor.set(TalonFXControlMode.Position, targetUnits, DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(positionRad, 0) / 12.0);
    SmartDashboard.putNumber("Target Pos (Native)", targetUnits);
    SmartDashboard.putNumber("PID Error (Native)", motor.getClosedLoopError());
  }

  public boolean isBottomedOut() {
    return motor.isRevLimitSwitchClosed() == 1;
  }

  public void moveAutomatic(ArmDirection direction, TrapezoidProfile.State state) {
    switch(direction) {
      case DOWN:
        moveDownAutomatic(state);
        break;
      case UP:
        moveUpAutomatic(state);
        break;
    }
  }

  public double sensorUnitsToRotations(double sensorUnits) {
    return (sensorUnits / TALON_UNITS_PER_REV) / GEARING;
  }

  public int radToSensorUnits(double rad) {
    return rotationsToSensorUnits(rad / (Math.PI * 2));
  }

  public int rotationsToSensorUnits(double rotations) {
    return (int) Math.round(rotations * GEARING * TALON_UNITS_PER_REV);
  }

  public int radsPerSecondToSensorUnits(double radsPerSecond) {
    double rotationsPerSecond = radsPerSecond / (Math.PI * 2);
    return rotationsToSensorUnits(rotationsPerSecond);
  }

  public enum ArmDirection {
    UP, DOWN;
  }
}
