// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmDirection;

public class ArmManualHoldCommand extends CommandBase {
  private static final double INCREMENT_RAD = Math.toRadians(10);
  private final Arm arm;
  private final ArmDirection direction;
  private Double holdRad;


  /** Creates a new ArmManualHoldCommand. */
  public ArmManualHoldCommand(Arm arm, ArmDirection direction) {
    this.arm = arm;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdRad = arm.getPositionRad() + ((direction == ArmDirection.UP ? 1 : -1) * INCREMENT_RAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.configurePID(direction);
    arm.holdPosition(holdRad, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopMoving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
