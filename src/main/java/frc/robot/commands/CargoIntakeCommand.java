// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.Cargo;
import frc.robot.subsystems.CargoManipulator;

public class CargoIntakeCommand extends CommandBase {
  private final CargoManipulator cargoManipulator;
  private final double intakeSpeed;

  /** Creates a new IntakeCommand. */
  public CargoIntakeCommand(CargoManipulator cargoManipulator, double intakeSpeed) {
    this.cargoManipulator = cargoManipulator;
    this.intakeSpeed = intakeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cargoManipulator);
  }

  public CargoIntakeCommand(CargoManipulator cargoManipulator) {
    this(cargoManipulator, 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cargoManipulator.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cargoManipulator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
