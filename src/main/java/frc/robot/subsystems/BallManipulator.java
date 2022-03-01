// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.BallManipulator.*;

public class BallManipulator extends SubsystemBase {
  private static final double INTAKE_SPEED = 0.5;
  private static final double SHOOT_SPEED = 1;
  private TalonSRX motor = new TalonSRX(MOTOR);


  /** Creates a new BallManipulator. */
  public BallManipulator() {
      motor.configFactoryDefault();
      motor.setInverted(true);
  }

  public void intake() {
    motor.set(ControlMode.PercentOutput, INTAKE_SPEED);
  }

  public void shoot() {
    motor.set(ControlMode.PercentOutput, -SHOOT_SPEED);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
