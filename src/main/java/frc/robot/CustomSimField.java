package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import java.util.HashMap;
import java.util.Map;

public class CustomSimField {
    public static final Pose2d FIELD_CENTER = new Pose2d(new Translation2d(16.4592 / 2, 8.2296 / 2), new Rotation2d());

    private final Field2d field2d = new Field2d();
    private RobotState currentState = null;
    private final Map<RobotState, FieldObject2d> robotStateMap = new HashMap<>();
    private FieldObject2d trajectoryDisplay;
    private Pose2d robotPosition = new Pose2d();

    public CustomSimField() {
        for (RobotState state : RobotState.values()) {
            FieldObject2d fieldObject = field2d.getObject("Robot " + state.getName());
            hideObject(fieldObject);
            robotStateMap.put(state, fieldObject);
        }
        hideObject(field2d.getRobotObject()); // Hide default Robot object
        setRobotState(RobotState.ARM_UP); // Start with arm up

        trajectoryDisplay = field2d.getObject("trajectory");
        hideObject(trajectoryDisplay);
    }

    public Field2d getField2d() {
        return field2d;
    }

    public Pose2d getRobotPosition() {
        return robotPosition;
    }

    public void setRobotPosition(Pose2d robotPosition) {
        this.robotPosition = robotPosition;
        if (currentState != null) {
            robotStateMap.get(currentState).setPose(robotPosition);
        }
    }

    public void setRobotState(RobotState state) {
        if (currentState != state) {
            // Hide all robot states except for new state
            robotStateMap.entrySet()
                    .stream()
                    .filter(entry -> entry.getKey() != state)
                    .forEach(entry -> hideObject(entry.getValue()));
            // Set robot position
            robotStateMap.get(state).setPose(robotPosition);

            currentState = state;
        }
    }

    private void hideObject(FieldObject2d fieldObject) {
        fieldObject.setPoses();
    }

    public void setTrajectory(Trajectory trajectory) {
        trajectoryDisplay.setTrajectory(trajectory);
    }

    public void clearTrajectory() {
        hideObject(trajectoryDisplay);
    }

    public enum RobotState {
        ARM_UP("Arm Up"),
        ARM_DOWN("Arm Down"),
        INTAKING("Intaking"),
        SHOOTING("Shooting");

        private final String name;

        RobotState(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }
}
