package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;

import java.util.List;
import java.util.function.Supplier;

public class ThreeBallAuto3 extends SequentialCommandGroup {
    private static final String FIRST_TRAJECTORY = "3_A1";
    private static final String SECOND_TRAJECTORY = "3_A2";
    private static final List<Cargo> cargos = List.of(Cargo.A, Cargo.B);
    private static final double SLOW_DOWN_RADIUS = Units.feetToMeters(2.0); // Radius around balls to slow down
    private static final double SLOW_DOWN_VELOCITY = 1.0;
    private static final double MAX_VELOCITY = 2.0;
    private static final double MAX_ACCEL = 0.5;
    private static final double LAST_BALL_DISTANCE_THRESHOLD = Units.feetToMeters(1.5);

    // https://www.desmos.com/calculator/akzhznta0w

    public ThreeBallAuto3(DriveTrain driveTrain, CargoManipulator cargoManipulator, Arm arm) {
        var reverseConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);
        Trajectory firstTrajectory = TrajectoryUtil.generateTrajectory(FIRST_TRAJECTORY, reverseConfig);

        var forwardConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        // Add slow down near balls
        cargos.forEach(cargo -> {
            forwardConfig.addConstraint(new EllipticalRegionConstraint(cargo.getPosition(), SLOW_DOWN_RADIUS * 2,
            SLOW_DOWN_RADIUS * 2, new Rotation2d(), new MaxVelocityConstraint(SLOW_DOWN_VELOCITY)));
        });

        Trajectory secondTrajectory = TrajectoryUtil.generateTrajectory(SECOND_TRAJECTORY, forwardConfig);

        if (firstTrajectory == null || secondTrajectory == null) {
            System.out.println("WARNING: Three ball auto trajectories failed to generate");
            return;
        }

        LastBallTracker lastBallTracker = new LastBallTracker(Cargo.B, driveTrain::getPose);

        addCommands(
            // Reset odometry
            new DriveResetOdometryCommand(driveTrain, firstTrajectory.getInitialPose()),
            // Shoot
            new CargoShootCommand(cargoManipulator).withTimeout(3.0),
            // Drive backwards while moving arm down
            new ParallelDeadlineGroup(
                TrajectoryUtil.createCommand(firstTrajectory, driveTrain),
                new ArmFirstDownCommand(arm)
            ),
            // Drive forwards while intake is running to pickup balls
            new ParallelDeadlineGroup(
                TrajectoryUtil.createCommand(secondTrajectory, driveTrain),
                // Hold arm down UNTIL ball is picked up
                new ArmHoldPositionCommand(arm, ArmPosition.DOWN)
                .until(lastBallTracker::hasPickedUpBall)
                .andThen(new ArmMoveToCommand(arm, ArmPosition.UP)),
                // Keep intaking
                new CargoIntakeCommand(cargoManipulator)
            ),
            // Shoot
            new CargoShootCommand(cargoManipulator)
        );
    }

    public static class LastBallTracker {
        private final Translation2d position;
        private final Supplier<Pose2d> poseSupplier;
        private boolean hasReached = false;
        private boolean hasLeft = false;

        public LastBallTracker(Cargo cargo, Supplier<Pose2d> poseSupplier) {
            this.position = cargo.getPosition();
            this.poseSupplier = poseSupplier;
        }

        // State logic
        private void update(Pose2d pose) {
            double distance = pose.getTranslation().getDistance(position);
            if (!hasReached && distance < LAST_BALL_DISTANCE_THRESHOLD) {
                hasReached = true;
            }
            if (hasReached && !hasLeft && distance > LAST_BALL_DISTANCE_THRESHOLD) {
                hasLeft = true;
            }
        }

        public boolean hasPickedUpBall() {
            update(poseSupplier.get());
            return hasReached && hasLeft;
        }
    }
}
