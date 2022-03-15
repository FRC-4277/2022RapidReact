package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BiConsumer;

import static frc.robot.Constants.DriveTrain.*;

public class CustomRamseteCommand extends RamseteCommand {
    public CustomRamseteCommand(DriveTrain driveTrain, Trajectory trajectory, boolean dependOnDrive) {
        super(trajectory, driveTrain::getPose, new RamseteController(), DRIVE_KINEMATICS, new BiConsumer<>() {
            private double previousLeftVelocityMPS, previousRightVelocityMPS;
            private double prevTimestamp;
            private boolean firstRun = true;
            private final Timer timer = new Timer();

            @Override
            public void accept(Double leftVelocityMPS, Double rightVelocityMPS) {
                double time = timer.get();
                if (firstRun) {
                    DifferentialDriveWheelSpeeds speeds = driveTrain.getWheelSpeeds();
                    previousLeftVelocityMPS = speeds.leftMetersPerSecond;
                    previousRightVelocityMPS = speeds.rightMetersPerSecond;

                    timer.reset();
                    timer.start();

                    prevTimestamp = time;
                    firstRun = false;
                }

                double dt = time - prevTimestamp;
                drive(leftVelocityMPS, previousLeftVelocityMPS, dt, DriveTrain.Side.LEFT);
                drive(rightVelocityMPS, previousRightVelocityMPS, dt, DriveTrain.Side.RIGHT);
            }

            private void drive(double velocity, double previousVelocity,
                               double dt, DriveTrain.Side side) {
                double acceleration = dt != 0 ? (velocity - previousVelocity) / dt : 0; // Avoid divide by 0
                double feedforwardVolts = MOTOR_FEEDFORWARD.calculate(velocity, acceleration);
                double feedforward = feedforwardVolts / MAX_BATTERY_V; // Normalize to 0..1
                if (HAS_ENCODERS && !RobotBase.isSimulation()) {
                    double nativeVelocity = driveTrain.metersToSensorUnits(velocity) / 10; // Convert to units/100ms
                    driveTrain.velocityDrive(side, nativeVelocity, feedforward);
                } else {
                    driveTrain.rawDriveSide(side, feedforward);
                }
            }
        }, dependOnDrive ? new Subsystem[] {driveTrain} : new Subsystem[] {});
    }
}
