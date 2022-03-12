// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmDirection;
import static frc.robot.Constants.Arm.*;

public class ArmAutoMoveCommand extends CommandBase {
  private Arm arm;
  private ArmDirection direction;
  private TrapezoidProfile trapezoidProfile;
  private Timer timer;

  /** Creates a new ArmManualMoveCommand. */
  public ArmAutoMoveCommand(Arm arm, ArmDirection direction) {
    this.arm = arm;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrapezoidProfile.State start = new TrapezoidProfile.State(arm.getPositionRad(), 0);
    TrapezoidProfile.State end;
    if (direction == ArmDirection.UP) {
      end = new TrapezoidProfile.State(Math.toRadians(DESIRED_DEGREES), 0);
    } else  {
      end = new TrapezoidProfile.State(0, 0);
    }
    trapezoidProfile = new TrapezoidProfile(TRAPEZOID_CONSTRAINTS, start, end);
    timer = new Timer();
    timer.reset();
    timer.start();
    System.out.println("START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= trapezoidProfile.totalTime()) {
      // Use trapezoid profile
      TrapezoidProfile.State state = trapezoidProfile.calculate(timer.get());
      state.position = Math.toRadians(DESIRED_DEGREES) - state.position;
      arm.moveAutomatic(direction, state);
      System.out.println("ARM POSITION TARGET: " + state.position + " AT t=" + timer.get());
    } else {
      System.out.println("HOLDING");
      // Hold desired position
      if (direction == ArmDirection.UP) {
        // PID hold at position
        arm.configurePID(ArmDirection.UP);
        arm.holdPosition(Math.toRadians(DESIRED_DEGREES), false);
      } else {
        // Have it perform its logic on how to hold down
        arm.moveDownAutomatic(null);
      }
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
    return false;
  }
}
