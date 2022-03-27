// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import static frc.robot.Constants.Arm.*;

/**
 * Command to run motion profile to move to a position, from any position
 */
public class ArmMoveToCommand extends CommandBase {
  /* Maximum extra time the command is allowed to run past the motion profile to get to target,
   if target is not already reached by the profile.
   Useful for making sure autonomous is timed properly
  */
  private static final double MAX_EXTRA_SECONDS = 0.5;

  private final Arm arm;
  private final ArmPosition position;
  private final boolean runForever;
  private final boolean allowOvertime;

  private TrapezoidProfile trapezoidProfile;
  private Timer timer;

  public ArmMoveToCommand(Arm arm, ArmPosition position, boolean runForever, boolean allowOvertime) {
    this.arm = arm;
    this.position = position;
    this.runForever = runForever;
    this.allowOvertime = allowOvertime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveToCommand(Arm arm, ArmPosition position) {
    this(arm, position, true, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create motion profile from current position to target position
    TrapezoidProfile.State start = arm.getTrapezoidState();
    TrapezoidProfile.State end = new TrapezoidProfile.State(Math.toRadians(position.getDegrees()), 0);
    trapezoidProfile = new TrapezoidProfile(TRAPEZOID_CONSTRAINTS, end, start);

    // Start timer
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TRUE in the if block = use motion profile
    // FALSE in the if block = hold target position as motion profile is done
    if (timer.get() <= trapezoidProfile.totalTime()) {
      // Use trapezoid profile
      TrapezoidProfile.State state = trapezoidProfile.calculate(timer.get());
      arm.moveToState(position, state);
    } else {
      arm.holdPosition(position);
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
    // Never finish if running forever
    if (runForever) {
      return false;
    }

    boolean profileDone = timer.get() > trapezoidProfile.totalTime();

    // Finish if over time is not allowed and motion profile is done
    if (!allowOvertime) {
      return profileDone;
    }

    // Finish if motion profile done & we're at target position
    if (profileDone && arm.isAtPosition(position)) {
      return true;
    }

    // Force finish if max over time is reached
    return timer.get() > (trapezoidProfile.totalTime()) + MAX_EXTRA_SECONDS;
  }
}
