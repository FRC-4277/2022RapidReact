package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.trajectory.LazyRamseteCommand;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.Map;

public class TwoBallAuto2 extends SequentialCommandGroup {
    private static final double MAX_VELOCITY = 2.0;
    private static final double MAX_ACCEL = 0.75;

    // https://www.desmos.com/calculator/cl6u6ckvkj

    private final Map<Cargo, Pose2d> STARTING_POSITIONS =
        Map.of(
            Cargo.A, new Pose2d(7.64, 1.986, new Rotation2d(4.712)),
            Cargo.B, new Pose2d(6.84, 2.89, new Rotation2d(3.57)),
            Cargo.D, new Pose2d(6.6131, 5.063, new Rotation2d(2.38))
        );

    private final Map<Cargo, Translation2d> PICKUP_POSITIONS =
        Map.of(
            Cargo.A, new Translation2d(7.63, 1.502),
            Cargo.B, new Translation2d(5.987, 2.5),
            Cargo.D, new Translation2d(5.61, 5.59)
        );

    private final Map<Cargo, Double> PICKUP_DISTANCE =
        Map.of(
            Cargo.A, 1.3 - Units.inchesToMeters(34.5),
            Cargo.B, 1.5,
            Cargo.D, 1.5
        );

    private final Map<Cargo, Pose2d> BEFORE_SHOOT_POSITIONS =
        Map.of(
            Cargo.A, new Pose2d(8.5,1.584, new Rotation2d(Math.PI)),
            Cargo.B, new Pose2d(5.7,3.548, new Rotation2d(5.3)),
            Cargo.D, new Pose2d(6.53, 6.25, new Rotation2d(4.1))
        );

    private final Map<Cargo, Pose2d> SHOOT_POSITIONS =
        Map.of(
            Cargo.A, new Pose2d(7.72, 2.82, new Rotation2d(1.1906147)),
            Cargo.B, new Pose2d(7.72, 2.82, new Rotation2d(1.1906147)),
            Cargo.D, new Pose2d(6.99, 4.59, new Rotation2d(5.8708))
        );

    public TwoBallAuto2(CargoManipulator cargoManipulator, DriveTrain driveTrain, Arm arm, Cargo cargo, Vision vision) {
        assert cargo.isCloseBall();

        Pose2d startingPosition = STARTING_POSITIONS.get(cargo);
        Pose2d pickupPosition = cargo.getPickupPose(PICKUP_POSITIONS.get(cargo));
        TrajectoryConfig config = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        // Have ending velocity be intake driving speed
        config.setEndVelocity(DriveTrain.convertPercentToVelocity(AutoBallPickupCommand.DRIVE_SPEED));
        Trajectory pickupTrajectory = TrajectoryUtil.generateTrajectory(startingPosition, pickupPosition, config);

        System.out.println(pickupPosition);

        TrajectoryConfig backConfig = TrajectoryUtil.createConfig(3, 2);
        backConfig.addConstraint(new CentripetalAccelerationConstraint(.5));

        addCommands(
            // Reset odometry
            new DriveResetOdometryCommand(driveTrain, startingPosition),
            // Initialize vision
            new VisionConfigureForAutoCommand(vision),
            // Move arm all the way down
            new ArmFirstDownCommand(arm).withTimeout(5.0),
            // Drive to pickup position with intake moving and arm is held down
            new ParallelDeadlineGroup(
                TrajectoryUtil.createCommand(pickupTrajectory, driveTrain),
                new CargoIntakeCommand(cargoManipulator),
                new ArmHoldPositionCommand(arm, ArmPosition.DOWN)
            ),
            // Pickup ball & drive a little further
            new AutoBallPickupCommand(driveTrain, cargoManipulator, arm, vision, PICKUP_DISTANCE.get(cargo)),
            // Drive back (three point turn), keep running intake & put arm up all at same time
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    // Drive backwards to BEFORE shoot position
                    new LazyRamseteCommand(driveTrain, () -> {
                        var config1 = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);
                        return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), BEFORE_SHOOT_POSITIONS.get(cargo), config1);
                    }, false),
                    // Drive forwards to shoot position
                    new LazyRamseteCommand(driveTrain, () -> {
                        var config1 = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
                        return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), SHOOT_POSITIONS.get(cargo), config1);
                    })
                ),
                new ArmMoveToCommand(arm, ArmPosition.UP, false, false),
                new CargoIntakeCommand(cargoManipulator)
            ),
            // Shoot
            new CargoShootCommand(cargoManipulator)
        );
    }
}
