package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class ResetOdometryCommand extends InstantCommand {
    public ResetOdometryCommand(DriveTrain driveTrain, Pose2d pose2d) {
        super(() -> driveTrain.resetOdometry(pose2d), driveTrain);
    }
}
