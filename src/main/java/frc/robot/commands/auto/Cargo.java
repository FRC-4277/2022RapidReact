package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum Cargo {
    // https://www.desmos.com/calculator/c4hopgbh2g
    A(true, new Translation2d(7.626, 0.68), new Translation2d(7.63, 1.246)),
    B(true, new Translation2d(5.32, 2.11), new Translation2d(5.87, 2.49)),
    D(true, new Translation2d(5.21, 6), new Translation2d(5.67, 5.537)),
    TERMINAL(false, new Translation2d(1.65, 1.434), new Translation2d(1.9, 1.643));

    private final boolean closeBall;
    private final Translation2d position;
    private final Translation2d pickupPosition;

    Cargo(boolean closeBall, Translation2d position, Translation2d pickupPosition) {
        this.closeBall = closeBall;
        this.position = position;
        this.pickupPosition = pickupPosition;
    }

    public boolean isCloseBall() {
        return closeBall;
    }

    public Translation2d getPosition() {
        return position;
    }

    public Translation2d getPickupPosition() {
        return pickupPosition;
    }

    /**
     * @return The pose where the robot will be to pickup the cargo
     */
    public Pose2d getPickupPose() {
        double x = pickupPosition.getX() - position.getX();
        double y = pickupPosition.getY() - position.getY();
        Rotation2d rotation = new Rotation2d(x, y);
        return new Pose2d(getPickupPosition(), rotation);
    }
}
