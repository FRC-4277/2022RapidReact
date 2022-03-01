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
  public static final double SPEED = 0.5;
  public static final double SLOW_ZONE_DEGREES = 10; // Slow down within 10 degrees of end
  public static final double SLOW_SPEED = 0.3;
  public static final double MOTION_MAGIC_THREHSOLD = 1750;
  private TalonFX motor = new TalonFX(MOTOR);

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
  
    motor.configMotionCruiseVelocity(CRUISE_VELOCITY_NATIVE);
    motor.configMotionAcceleration(CRUISE_ACCELERATION_NATIVE);
    //motor.configMotionSCurveStrength(S_CURVE_SMOOTHING);

    motor.config_kP(0, PID_P);
    motor.config_kI(0, PID_I);
    motor.config_kD(0, PID_D);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Native", motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Degrees", getPositionDegrees());
    SmartDashboard.putNumber("Arm Rad", Math.toRadians(getPositionDegrees()));
  }

  public void moveDownManual() {
    motor.set(ControlMode.PercentOutput, -SPEED);
  }

  public void moveDownManualSlow() {
    motor.set(ControlMode.PercentOutput, -SLOW_SPEED);
  }

  public void moveUpManual() {
    motor.set(ControlMode.PercentOutput, SPEED);
  }

  public void moveUp(double speed) {
    motor.set(ControlMode.PercentOutput, Math.abs(speed));
  }

  public void moveUpManualSlow() {
    motor.set(ControlMode.PercentOutput, SLOW_SPEED);
  }

  public void moveManual(ArmDirection direction) {
    switch(direction) {
      case DOWN:
        moveDownManual();
        break;
      case UP:
        moveUpManual();
        break;
    }
  }

  public void stopMoving() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void moveDownAutomatic(TrapezoidProfile.State state) {
    if (getPositionDegrees() <= 10) {
      System.out.println("Move down auto: slow");
      moveDownManualSlow();
    } else {
      double positionSensorUnits = radToSensorUnits(state.position);
      System.out.println("Move down auto: " + positionSensorUnits);
      System.out.println("Current pos: " + motor.getSelectedSensorPosition());
      motor.set(TalonFXControlMode.Position, positionSensorUnits, DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(state.position, state.velocity) / 12.0);
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
    double positionSensorUnits = radToSensorUnits(state.position);
    motor.set(TalonFXControlMode.Position, positionSensorUnits, DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(state.position, state.velocity) / 12.0);
    System.out.println("Move up auto: " + positionSensorUnits);
    System.out.println("Current pos: " + motor.getSelectedSensorPosition());
  }

  public void holdPosition(double positionRad) {
    motor.set(TalonFXControlMode.Position, radToSensorUnits(positionRad), DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(positionRad, 0) / 12.0);
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

  public static enum ArmDirection {
    UP, DOWN;
  }
}
