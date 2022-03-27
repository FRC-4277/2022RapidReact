package frc.robot.commands.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class TrajectoryUtil {

    /**
     * Generate trajectory from file
     *
     * @param pathFileName Specify the {THIS} in src/main/deploy/paths/{THIS}.wpilib.json.
     * @return Trajectory from loaded file
     */
    public static Trajectory generateTrajectory(String pathFileName) {
        String path = "paths/" + pathFileName + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            return edu.wpi.first.math.trajectory.TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + path, e.getStackTrace());
        }
        return null;
    }

    public static Trajectory generateTrajectory(String pathName, TrajectoryConfig config) {
        try {
            config.setKinematics(Constants.DriveTrain.DRIVE_KINEMATICS);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    WaypointReader.getControlVectors(pathName), config);

            // Shuffleboard
            RobotContainer.getInstance().addAutoTime(pathName, trajectory.getTotalTimeSeconds());

            return trajectory;
        } catch (Exception e) {
            System.err.println("Failed to generate custom trajectory for path name " + pathName);
            e.printStackTrace();
        }

        return null;
    }

    public static TrajectoryConfig createConfig(double maxVel, double maxAccel) {
        return new TrajectoryConfig(maxVel, maxAccel);
    }

    public static TrajectoryConfig createConfig(double maxVel, double maxAccel, boolean reversed) {
        TrajectoryConfig config = createConfig(maxVel, maxAccel);
        config.setReversed(reversed);
        return config;
    }

    public static TrajectoryConfig createConfig(double maxVel, double maxAccel, double maxCentripetalAccel) {
        var config = new TrajectoryConfig(maxVel, maxAccel);
        config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccel));
        return config;
    }

    public static TrajectoryConfig createConfig(double maxVel, double maxAccel,
                                                boolean reversed, double maxCentripetalAccel) {
        var config = new TrajectoryConfig(maxVel, maxAccel);
        config.setReversed(reversed);
        config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccel));
        return config;
    }

    public static Trajectory generateTransformTrajectory(Pose2d start, Transform2d transform2d, TrajectoryConfig config) {
        Pose2d end = start.plus(transform2d);
        return TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
    }

    public static Trajectory generateTrajectory(Pose2d start, Pose2d end, TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
    }

    public static Trajectory generateTranslateTrajectory(Pose2d start, Translation2d translation2d, TrajectoryConfig config) {
        return generateTransformTrajectory(start, new Transform2d(translation2d, new Rotation2d()), config);
    }

    public static Trajectory generateStraightTrajectory(Pose2d start, TrajectoryConfig config, double distanceM) {
        if (distanceM < 0) {
            config.setReversed(true);
        }
        return generateTranslateTrajectory(start, new Translation2d(distanceM, 0), config);
    }

    public static CustomRamseteCommand createCommand(Trajectory trajectory, DriveTrain driveTrain) {
        return new CustomRamseteCommand(driveTrain, trajectory, true);
    }
}
