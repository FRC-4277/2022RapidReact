// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveJoystickCommand extends CommandBase {
  private static final double MAX_ACCELERATION = 2.0;
  private final DriveTrain driveTrain;
  private final Joystick joystick;
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(MAX_ACCELERATION);

  /** Creates a new JoystickDriveCommand. */
  public DriveJoystickCommand(DriveTrain driveTrain, Joystick joystick) {
    this.driveTrain = driveTrain;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = joystick.getZ();
    boolean turnInPlace = joystick.getRawButton(1);
    // Half speed rotation when turning in place
    if (turnInPlace) {
      rotation *= 0.5;
    }

    double speed = -joystick.getY();
    // Limit accel if controller is > 20% either way
    if (Math.abs(speed) > 0.20) {
      speed = speedLimiter.calculate(speed);
    }
    
    driveTrain.joystickDrive(speed, rotation, turnInPlace);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
