// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CustomSimField;
import frc.robot.commands.ArmFirstDownCommand;
import frc.robot.commands.DriveResetOdometryCommand;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveOnlyAuto0 extends SequentialCommandGroup {
  private static final double VELOCITY = 0.5;
  private static final double ACCEL = 0.5;
  private static final double DISTANCE = Units.feetToMeters(6.5);

  /** Creates a new MoveOnlyAuto. */
  public MoveOnlyAuto0(DriveTrain driveTrain, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var start = !RobotBase.isSimulation() ? new Pose2d() : CustomSimField.FIELD_CENTER;
    var config = TrajectoryUtil.createConfig(VELOCITY, ACCEL);
    Trajectory trajectory = TrajectoryUtil.generateStraightTrajectory(start, config, -DISTANCE);
    addCommands(
      new DriveResetOdometryCommand(driveTrain, start),
      TrajectoryUtil.createCommand(trajectory, driveTrain),
      new ArmFirstDownCommand(arm)
    );
  }
}
