// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmFirstDownCommand;
import frc.robot.commands.ArmHoldPositionCommand;
import frc.robot.commands.ArmMoveToCommand;
import frc.robot.commands.CargoShootCommand;
import frc.robot.commands.trajectory.LazyRamseteCommand;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;

import java.util.Map;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto2 extends SequentialCommandGroup {
  // Speed constraints. 1 = First trajectory to ball, 2 = trajectory back to shoot
  private static final double VELOCITY_1 = 2.0;
  private static final double ACCEL_1 = 2.0;
  private static final double CENTRIPETAL_1 = 2.0;
  private static final double VELOCITY_2 = 2.0;
  private static final double ACCEL_2 = 2.0;
  private static final double CENTRIPETAL_2 = 2.0;

  // https://www.desmos.com/calculator/oyqxjvsfcg
  private final Map<Cargo, Pose2d> STARTING_POSITIONS =
    Map.of(
        Cargo.A, new Pose2d(7.92, 2.94, new Rotation2d(5.903)),
        Cargo.B, new Pose2d(7.62, 3.07, new Rotation2d(5.903)),
        Cargo.D, new Pose2d(6.97, 4.83, new Rotation2d(4.3))
    );

  private final Map<Cargo, Translation2d> PICKUP_POSITIONS =
    Map.of(
        Cargo.A, new Translation2d(7.63, 1.246),
        Cargo.B, new Translation2d(5.87, 2.49),
        Cargo.D, new Translation2d(5.67, 5.537)
    );

  /** Creates a new TwoBallAuto2. */
  public TwoBallAuto2(CargoManipulator cargoManipulator, DriveTrain driveTrain, Arm arm, Cargo cargo) {
    assert cargo.isCloseBall();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Pose2d startingPosition = STARTING_POSITIONS.get(cargo);
    Pose2d endPosition = cargo.getPickupPose(PICKUP_POSITIONS.get(cargo));
    Trajectory firstTrajectory = TrajectoryUtil.generateTrajectory(startingPosition, endPosition,
            TrajectoryUtil.createConfig(VELOCITY_1, ACCEL_1, true));

    addCommands(
      // Shoot
      new CargoShootCommand(cargoManipulator).withTimeout(2.0),
      // Drive to ball & move arm down in parallel
      new ParallelDeadlineGroup(
        TrajectoryUtil.createCommand(firstTrajectory, driveTrain),
        new SequentialCommandGroup(
          // Wait before moving arm down
          new WaitCommand(2.0),
          new ArmFirstDownCommand(arm)
        )
      ),
      // Pickup ball
      new AutoBallPickupCommand(driveTrain, cargoManipulator, arm),
      // Drive back to start & move arm up in parallel
      new ParallelDeadlineGroup(
        new LazyRamseteCommand(driveTrain, () ->
          TrajectoryUtil.generateTrajectory(driveTrain.getPose(), startingPosition,
            TrajectoryUtil.createConfig(VELOCITY_2, ACCEL_2, true))
        ),
        new ArmMoveToCommand(arm, ArmPosition.UP)
      ),
      // Hold arm & shoot
      new ParallelCommandGroup(
        new ArmHoldPositionCommand(arm, ArmPosition.UP),
        new CargoShootCommand(cargoManipulator)
      )
    );
  }
}
