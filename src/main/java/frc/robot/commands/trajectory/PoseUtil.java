package frc.robot.commands.trajectory;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseUtil {
    public static double getDistance(Pose2d pose1, Pose2d pose2) {
        return Math.hypot(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY());
    }
}
