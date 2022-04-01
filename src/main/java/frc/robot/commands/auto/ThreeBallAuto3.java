package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.trajectory.TrajectoryUtil;
import frc.robot.subsystems.DriveTrain;

public class ThreeBallAuto3 extends SequentialCommandGroup {
    private static final String FIRST_TRAJECTORY_FORMAT = "3_%s1";
    private static final String SECOND_TRAJECTORY_FORMAT = "3_%s2";
    private static final double MAX_VELOCITY = 0.75;
    private static final double MAX_ACCEL = 0.250;

    public ThreeBallAuto3(DriveTrain driveTrain, Cargo cargo) {
        var reverseConfig = TrajectoryUtil.createConfig(MAX_VELOCITY, MAX_ACCEL, true);
        Trajectory firstTrajectory = TrajectoryUtil.generateTrajectory(
                String.format(FIRST_TRAJECTORY_FORMAT, cargo.name()), reverseConfig);
        

        addCommands();
    }
}
