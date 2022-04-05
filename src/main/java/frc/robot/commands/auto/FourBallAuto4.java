package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.trajectory.LazyRamseteCommand;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.List;

public class FourBallAuto4 extends SequentialCommandGroup {
    private static final String SECOND_TRAJECTORY = "4_A1"; // Pickup balls
    //private static final String THIRD_TRAJECTORY = "4_A2"; // Reverse to turn around
    //private static final String FOURTH_TRAJECTORY = "4_A3"; // Forward to shooting position
    private static final double MAX_VELOCITY = 4.0;
    private static final double MAX_ACCEL = 2.5;
    private static final double SLOW_DOWN_RADIUS = Units.feetToMeters(2.0); // Radius around balls to slow down
    private static final double SLOW_DOWN_VELOCITY = 2.0;

    private static final Pose2d START_POSE = new Pose2d(7.64, 1.986, new Rotation2d(4.712));
    private static final Translation2d FIRST_PICKUP_TRANSLATION = new Translation2d(7.63, 1.502);
    private static final double FIRST_PICKUP_DISTANCE = 1.3 - Units.inchesToMeters(36);
    // Pose to go to after shooting first 2
    private static final Pose2d INTERMEDIATE_POSE = new Pose2d(8.37,1.93, new Rotation2d(Math.PI));
    private static final Pose2d SHOOTING_POSE = new Pose2d(7.82, 2.67, new Rotation2d(1.1906147));
    // Intermediate pose 2
    private static final Pose2d INTERMEDIATE_TWO_POSE = new Pose2d(7.61, 1.9, new Rotation2d(Math.PI/2));
    private static final Pose2d INTERMEDIATE_THREE_POSE = new Pose2d(7.6, 1.7, new Rotation2d(Math.PI/2));

    // https://www.desmos.com/calculator/sjpin0nfmk

    public FourBallAuto4(DriveTrain driveTrain, CargoManipulator cargoManipulator, Arm arm, Vision vision) {
        var forwardConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        var forwardWithBallsConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        // Add slow down near balls
        List.of(Cargo.B, Cargo.TERMINAL).forEach(cargo ->
                forwardWithBallsConfig.addConstraint(new EllipticalRegionConstraint(cargo.getPosition(),
                        SLOW_DOWN_RADIUS * 2, SLOW_DOWN_RADIUS * 2, new Rotation2d(),
                        new MaxVelocityConstraint(SLOW_DOWN_VELOCITY))));

        var reverseConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);

        Trajectory firstTrajectory = TrajectoryUtil.generateTrajectory(START_POSE, Cargo.A.getPickupPose(FIRST_PICKUP_TRANSLATION), forwardConfig);

        Trajectory pickupTrajectory = TrajectoryUtil.generateTrajectory(SECOND_TRAJECTORY, forwardWithBallsConfig);

        Timer timer = new Timer();

        addCommands(
            // Reset odometry
            new DriveResetOdometryCommand(driveTrain, firstTrajectory.getInitialPose()),
            // Initialize vision
            new VisionConfigureForAutoCommand(vision),
            // Timer start,
            new InstantCommand(() -> {
                timer.reset();
                timer.start();
            }),
            // Move arm all the way down
            new ParallelDeadlineGroup(
                new ParallelDeadlineGroup(
                        TrajectoryUtil.createCommand(firstTrajectory, driveTrain),
                        new CargoIntakeCommand(cargoManipulator)
                ),
                new ArmFirstDownCommand(arm)
            ),
            new AutoBallPickupCommand(driveTrain, cargoManipulator, arm, vision, FIRST_PICKUP_DISTANCE),
            // Drive back (three point turn)
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        // Drive backwards to BEFORE shoot position
                        new LazyRamseteCommand(driveTrain, () -> {
                            var config1 = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);
                            config1.setEndVelocity(0.5);
                            return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), INTERMEDIATE_POSE, config1);
                        }, false),
                        // Drive forwards to shoot position
                        new LazyRamseteCommand(driveTrain, () -> {
                            var config1 = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
                            config1.setEndVelocity(1.5);
                            return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), SHOOTING_POSE, config1);
                        })
                ),
                new ArmMoveToCommand(arm, Arm.ArmPosition.UP, false, false),
                new CargoIntakeCommand(cargoManipulator)
            ),
            // Shoot
            new CargoShootCommand(cargoManipulator).withTimeout(0.75),
            // Drive back to intermediate 2 w/ arm down
            new ParallelDeadlineGroup(
                new LazyRamseteCommand(driveTrain, () -> {
                    var config1 = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);
                    config1.setEndVelocity(1.0);
                    return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), INTERMEDIATE_TWO_POSE, config1);
                }),
                new CargoIntakeCommand(cargoManipulator),
                new ArmHoldPositionCommand(arm, Arm.ArmPosition.DOWN)
            ),
            // Drive path to pickup balls
            new ParallelDeadlineGroup(
                TrajectoryUtil.createCommand(pickupTrajectory, driveTrain),
                new CargoIntakeCommand(cargoManipulator),
                new ArmHoldPositionCommand(arm, Arm.ArmPosition.DOWN)
            ),
            // Drive back to intermediate 3
            new ParallelDeadlineGroup(
                new LazyRamseteCommand(driveTrain, () -> {
                    TrajectoryConfig config1 = TrajectoryUtil.createConfig(4.0, 3.0, true);
                    config1.setEndVelocity(1.5);
                    return TrajectoryUtil.generateTrajectory(driveTrain.getPose(),
                            List.of(Cargo.B.getPosition()),
                            INTERMEDIATE_THREE_POSE, config1);
                }),
                new CargoIntakeCommand(cargoManipulator),
                new ArmMoveToCommand(arm, Arm.ArmPosition.UP)
            ),
            // Drive back & shoot
            new ParallelDeadlineGroup(
                    new LazyRamseteCommand(driveTrain, () -> {
                        TrajectoryConfig config1 = TrajectoryUtil.createConfig(4.0, 3.0);
                        config1.setEndVelocity(2.0);
                        return TrajectoryUtil.generateTrajectory(driveTrain.getPose(), SHOOTING_POSE, config1);
                    }),
                new CargoIntakeCommand(cargoManipulator),
                new ArmHoldPositionCommand(arm, Arm.ArmPosition.UP)
            ),
            // Shoot & print time at shoot,
            new InstantCommand(() -> System.out.println("Shooting at " + timer.get())),
            new CargoShootCommand(cargoManipulator)
        );
    }
}