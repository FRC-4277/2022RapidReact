package frc.robot.commands.trajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CustomSimField;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BiConsumer;

import static frc.robot.Constants.DriveTrain.*;

/**
 * A subclass of RamseteCommand that utilizes velocity PID on our TalonFXs
 */
public class CustomRamseteCommand extends RamseteCommand {
    private static final double SIM_TIME_EPSILON = 0.01;

    public CustomRamseteCommand(DriveTrain driveTrain, Trajectory trajectory, boolean dependOnDrive) {
        super(trajectory, driveTrain::getPose, new RamseteController(), DRIVE_KINEMATICS, new BiConsumer<>() {
            private double previousLeftVelocityMPS, previousRightVelocityMPS;
            private Double previousTimestamp;
            private boolean firstRun = true;
            private final Timer timer = new Timer();
            private boolean simTrajectoryHidden = false;

            @Override
            public void accept(Double leftVelocityMPS, Double rightVelocityMPS) {
                double time = timer.get();

                if (previousTimestamp != null && time < previousTimestamp) {
                    firstRun = true;
                }

                if (firstRun) {
                    DifferentialDriveWheelSpeeds speeds = driveTrain.getWheelSpeeds();
                    previousLeftVelocityMPS = speeds.leftMetersPerSecond;
                    previousRightVelocityMPS = speeds.rightMetersPerSecond;

                    timer.reset();
                    timer.start();

                    previousTimestamp = time;
                    firstRun = false;

                    // Simulation
                    if (RobotBase.isSimulation()) {
                        CustomSimField simField = RobotContainer.getInstance().getSimField();
                        if (simField != null) {
                            simField.setTrajectory(trajectory);
                        }
                    }
                }

                double dt = previousTimestamp != null ? time - previousTimestamp : 0;
                drive(leftVelocityMPS, previousLeftVelocityMPS, dt, DriveTrain.Side.LEFT);
                drive(rightVelocityMPS, previousRightVelocityMPS, dt, DriveTrain.Side.RIGHT);

                previousLeftVelocityMPS = leftVelocityMPS;
                previousRightVelocityMPS = rightVelocityMPS;
                previousTimestamp = time;

                /*// Simulation - hide trajectory at end
                if (RobotBase.isSimulation() && !simTrajectoryHidden &&
                        (Math.abs(trajectory.getTotalTimeSeconds() - time) <= SIM_TIME_EPSILON)) {

                    CustomSimField simField = RobotContainer.getInstance().getSimField();
                    if (simField != null) {
                        simField.clearTrajectory();
                    }
                    simTrajectoryHidden = true;
                }*/
            }

            private void drive(double velocity, double previousVelocity,
                               double dt, DriveTrain.Side side) {
                double acceleration = dt != 0 ? (velocity - previousVelocity) / dt : 0; // Avoid divide by 0
                double feedforwardVolts = MOTOR_FEEDFORWARD.calculate(velocity, acceleration);
                double feedforward = feedforwardVolts / MAX_BATTERY_V; // Normalize to 0..1
                if (HAS_ENCODERS && !RobotBase.isSimulation()) {
                    double nativeVelocity = ((double) driveTrain.metersToSensorUnits(velocity)) / 10; // Convert to units/100ms
                    driveTrain.velocityDrive(side, nativeVelocity, feedforward);
                } else {
                    driveTrain.rawDriveSide(side, feedforward);
                }
            }
        }, dependOnDrive ? new Subsystem[] {driveTrain} : new Subsystem[] {});
    }
}
