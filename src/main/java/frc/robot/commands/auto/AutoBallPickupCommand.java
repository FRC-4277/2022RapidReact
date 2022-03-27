// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.trajectory.PoseUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;

public class AutoBallPickupCommand extends CommandBase {
  private static final double DRIVE_SPEED = 0.175;
  private static final double DEFAULT_MAX_DISTANCE = 0.70; // meters
  private static final double DEFAULT_MAX_TIME = 1.0; // seconds

  private final DriveTrain driveTrain;
  private final CargoManipulator cargoManipulator;
  private final Arm arm;
  private final double maxDistance;
  private final double maxTime;

  private Pose2d startPose;
  private Timer timer;

  /** Creates a new AutoBallPickupCommand.
   * @param driveTrain
   * @param cargoManipulator
   * @param arm
   * @param distance In meters
   * @param maxTime   */
  public AutoBallPickupCommand(DriveTrain driveTrain, CargoManipulator cargoManipulator, Arm arm, double distance, double maxTime) {
    this.driveTrain = driveTrain;
    this.cargoManipulator = cargoManipulator;
    this.arm = arm;
    this.maxDistance = distance;
    this.maxTime = maxTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, cargoManipulator);
  }

  public AutoBallPickupCommand(DriveTrain driveTrain, CargoManipulator cargoManipulator, Arm arm) {
    this(driveTrain, cargoManipulator, arm, DEFAULT_MAX_DISTANCE, DEFAULT_MAX_TIME);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose = driveTrain.getPose();
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive
    double leftSpeed = DRIVE_SPEED;
    double rightSpeed = DRIVE_SPEED; // todo : ADD PHOTON VISION adjustment
    driveTrain.rawDrive(leftSpeed, rightSpeed);
    // hold arm down && intake
    arm.holdPosition(ArmPosition.DOWN);
    cargoManipulator.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
    cargoManipulator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finish if timer time is over max time OR distance is over max distance
    return timer.get() > maxTime ||
            (PoseUtil.getDistance(startPose, driveTrain.getPose()) > maxDistance);
  }
}
