// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

import static frc.robot.Constants.Arm.*;

/**
 * The command that will run in autonomous to move the arm down for the first time
 */
public class ArmFirstDownCommand extends CommandBase {
  private final Arm arm;
  private TrapezoidProfile trapezoidProfile;
  private Timer timer;

  /** Creates a new ArmFirstDownCommand. */
  public ArmFirstDownCommand(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create motion profile from current position to down position (0)
    TrapezoidProfile.State start = arm.getTrapezoidState();
    // Have the end position be a few degrees above 0 just in case the arm isn't in perfect start position
    TrapezoidProfile.State end = new TrapezoidProfile.State(Math.toRadians(Arm.SLOW_ZONE_DEGREES / 2), 0);
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 2), end, start);

    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= trapezoidProfile.totalTime()) {
      // Use trapezoid profile
      TrapezoidProfile.State state = trapezoidProfile.calculate(timer.get());
      arm.moveToState(ArmPosition.DOWN, state);
    } else {
      // Trapezoid profile done, now just hold at bottom
      arm.holdPosition(ArmPosition.DOWN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopMoving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // No need to stop, it will stop itself with limit switch
    return arm.isAtPosition(ArmPosition.DOWN) || (RobotBase.isSimulation() && timer.hasElapsed(trapezoidProfile.totalTime()));
  }
}
