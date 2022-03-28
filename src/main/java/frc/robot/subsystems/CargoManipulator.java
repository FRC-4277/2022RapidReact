// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomSimField;
import frc.robot.CustomSimField.RobotState;

import static frc.robot.Constants.BallManipulator.*;

public class CargoManipulator extends SubsystemBase {
  private static final double INTAKE_SPEED = 0.5;
  private static final double SHOOT_SPEED = 1;
  private final TalonSRX motor = new TalonSRX(MOTOR);

  private final CustomSimField simField;
  private RobotState lastState;

  public CargoManipulator(CustomSimField simField) {
    this.simField = simField;
    motor.configFactoryDefault();
    motor.setInverted(true);
  }

  public void intake() {
    motor.set(ControlMode.PercentOutput, INTAKE_SPEED);
    if (RobotBase.isSimulation()) {
      simField.setRobotState(lastState = RobotState.INTAKING);
    }
  }

  public void shoot() {
    motor.set(ControlMode.PercentOutput, -SHOOT_SPEED);
    if (RobotBase.isSimulation()) {
      simField.setRobotState(lastState = RobotState.SHOOTING);
    }
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
    if (RobotBase.isSimulation()) {
      if (lastState == RobotState.INTAKING) {
        simField.setRobotState(RobotState.ARM_DOWN);
      } else if (lastState == RobotState.SHOOTING){
        simField.setRobotState(RobotState.ARM_UP);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
