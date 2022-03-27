package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum Cargo {
    // https://www.desmos.com/calculator/c4hopgbh2g
    A(true, new Translation2d(7.626, 0.68)),
    B(true, new Translation2d(5.32, 2.11)),
    D(true, new Translation2d(5.21, 6)),
    TERMINAL(false, new Translation2d(1.65, 1.434));

    private final boolean closeBall;
    private final Translation2d position;

    Cargo(boolean closeBall, Translation2d position) {
        this.closeBall = closeBall;
        this.position = position;
    }

    public boolean isCloseBall() {
        return closeBall;
    }

    public Translation2d getPosition() {
        return position;
    }

    /**
     * @return The pose where the robot will be to pickup the cargo
     */
    public Pose2d getPickupPose(Translation2d pickupPosition) {
        double x = pickupPosition.getX() - position.getX();
        double y = pickupPosition.getY() - position.getY();
        Rotation2d rotation = new Rotation2d(x, y);
        return new Pose2d(pickupPosition, rotation);
    }
}
