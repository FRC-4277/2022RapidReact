// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class LazyRamseteCommand extends CommandBase {
  private final DriveTrain driveTrain;
  private final Supplier<Trajectory> trajectorySupplier;
  private final boolean stopAtEnd;
  private RamseteCommand ramseteCommand;

  /** Creates a new LazyRamseteCommand. */
  public LazyRamseteCommand(DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier, boolean stopDriveAtEnd) {
    this.driveTrain = driveTrain;
    this.trajectorySupplier = trajectorySupplier;
    this.stopAtEnd = stopDriveAtEnd;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public LazyRamseteCommand(DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier) {
    this(driveTrain, trajectorySupplier, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ramseteCommand == null) {
      ramseteCommand = new CustomRamseteCommand(driveTrain, trajectorySupplier.get(), false);
      ramseteCommand.initialize();
      return;
    }
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopAtEnd) {
      driveTrain.stopDrive();
    }
    if (ramseteCommand != null) {
      ramseteCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand != null && ramseteCommand.isFinished();
  }
}
