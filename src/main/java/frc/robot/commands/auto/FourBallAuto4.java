package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmFirstDownCommand;
import frc.robot.commands.CargoShootCommand;
import frc.robot.commands.DriveResetOdometryCommand;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.DriveTrain;

public class FourBallAuto4 extends SequentialCommandGroup {
    private static final String SECOND_TRAJECTORY = "4_A1"; // Pickup balls
    //private static final String THIRD_TRAJECTORY = "4_A2"; // Reverse to turn around
    //private static final String FOURTH_TRAJECTORY = "4_A3"; // Forward to shooting position
    private static final double MAX_VELOCITY = 2.0;
    private static final double MAX_ACCEL = 0.75;

    private static final Pose2d START_POSE = new Pose2d(7.64, 1.986, new Rotation2d(4.712));
    private static final Translation2d FIRST_PICKUP_TRANSLATION = new Translation2d(7.63, 1.502);
    private static final double FIRST_PICKUP_DISTANCE = 1.3 - Units.inchesToMeters(36);
    // Pose to go to after shooting first 2
    private static final Pose2d INTERMEDIATE_POSE = new Pose2d(8.5,1.584, new Rotation2d(Math.PI));
    private static final Pose2d SHOOTING_POSE = new Pose2d(7.84, 2.82, new Rotation2d(1.1906147));

    public FourBallAuto4(DriveTrain driveTrain, CargoManipulator cargoManipulator, Arm arm) {
        var forwardConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        var forwardWithBallsConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL);
        // Add slow down near balls

        var reverseConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);

        Trajectory firstTrajectory = TrajectoryUtil.generateTrajectory(START_POSE, INTERMEDIATE_POSE, reverseConfig);
        Trajectory pickupTrajectory = TrajectoryUtil.generateTrajectory(SECOND_TRAJECTORY, forwardConfig);
        Trajectory afterPickupTrajectory = TrajectoryUtil.generateTrajectory(THIRD_TRAJECTORY, reverseConfig);
        Trajectory backToShootTrajectory = TrajectoryUtil.generateTrajectory(FOURTH_TRAJECTORY, forwardConfig);

        if (firstTrajectory == null || pickupTrajectory == null ||
                afterPickupTrajectory == null || backToShootTrajectory == null) {
            System.out.println("WARNING: Three ball auto trajectories failed to generate");
            return;
        }

        addCommands(
            // Reset odometry
            new DriveResetOdometryCommand(driveTrain, firstTrajectory.getInitialPose())
        );
    }
}
