// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
  public static final double SPEED = 0.25;
  private TalonFX motor = new TalonFX(MOTOR);

  /** Creates a new Arm. */
  public Arm() {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.Clockwise);
    motor.setNeutralMode(NeutralMode.Brake);
  
    motor.configMotionCruiseVelocity(CRUISE_VELOCITY_NATIVE);
    motor.configMotionAcceleration(CRUISE_ACCELERATION_NATIVE);
    motor.configMotionSCurveStrength(S_CURVE_SMOOTHING);

    motor.config_kP(0, PID_P);
    motor.config_kI(0, PID_I);
    motor.config_kD(0, PID_D);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveDownManual() {
    motor.set(ControlMode.PercentOutput, SPEED);
  }

  public void moveUpManual() {
    motor.set(ControlMode.PercentOutput, -SPEED);
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

  public void moveDownAutomatic() {
    int setpoint = rotationsToSensorUnits(DESIRED_ROTATIONS);
    motor.set(ControlMode.MotionMagic, setpoint);
  }

  public void moveUpAutomatic() {
    motor.set(ControlMode.MotionMagic, 0);
  }

  public void moveAutomatic(ArmDirection direction) {
    switch(direction) {
      case DOWN:
        moveDownAutomatic();
        break;
      case UP:
        moveUpAutomatic();
        break;
    }
  }

  public double sensorUnitsToRotations(int sensorUnits) {
    return ((double) sensorUnits / TALON_UNITS_PER_REV) / GEARING;
  }

  public int rotationsToSensorUnits(double rotations) {
    return (int) Math.round(rotations * GEARING * TALON_UNITS_PER_REV);
  }

  public static enum ArmDirection {
    UP, DOWN;
  }
}
