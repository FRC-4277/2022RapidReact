package frc.robot.commands.trajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Path;

public class TrajectoryUtil {

    /**
     * Generate trajectory from file
     *
     * @param pathFileName Specify the {THIS} in src/main/deploy/paths/{THIS}.wpilib.json.
     * @return Trajectory from loaded file
     */
    public Trajectory generateTrajectory(String pathFileName) {
        String path = "paths/" + pathFileName + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            return edu.wpi.first.math.trajectory.TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + path, e.getStackTrace());
        }
        return null;
    }

    public Trajectory generateTrajectory(String pathName, TrajectoryConfig config) {
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

    public TrajectoryConfig createConfig(double maxVel, double maxAccel, double maxCentripetalAccel) {
        var config = new TrajectoryConfig(maxVel, maxAccel);
        config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccel));
        return config;
    }
}
