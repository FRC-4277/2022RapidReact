// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.DriveTrain.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(FRONT_LEFT);
  private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(FRONT_RIGHT);
  private final WPI_TalonFX backLeftMotor = new WPI_TalonFX(BACK_LEFT);
  private final WPI_TalonFX backRightMotor = new WPI_TalonFX(BACK_RIGHT);
  private final List<WPI_TalonFX> motors = List.of(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
  private final List<WPI_TalonFX> leftMotors = List.of(frontLeftMotor, backLeftMotor);
  private final List<WPI_TalonFX> rightMotors = List.of(frontRightMotor, backRightMotor);

  private final AHRS ahrs = new AHRS();
  private double yawOffset = 0;

  private final MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  private final DifferentialDriveOdometry odometry;
  private final DifferentialDrivetrainSim simulator = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2),
          GEARING,
          7.5, // MOI in kg m^2 (not sure real value)
          Units.lbsToKilograms(130),
          WHEEL_DIAMETER_M,
          TRACK_WIDTH_M,
          //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
          VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
  );

  private ShuffleboardTab autoTab;
  private NetworkTableEntry positionEntry;

  // State variables
  private boolean odometryEnabled = true;

  public DriveTrain(ShuffleboardTab autoTab) {
    this.autoTab = autoTab;
    positionEntry = autoTab.add("Position", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1)
      .withSize(2, 1)
    .getEntry();

    drive.setSafetyEnabled(false);
    //this code resets the motors everytime it runs.
    motors.forEach(motor -> {
      motor.configFactoryDefault();
      motor.config_kP(0, PID_P);
      motor.config_kI(0, PID_I);
      motor.config_kD(0, PID_D);
    });

    frontLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    frontRightMotor.setInverted(TalonFXInvertType.Clockwise);
    backLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    backRightMotor.setInverted(TalonFXInvertType.Clockwise);

    // Auto driving
    resetEncoders();
    zeroHeading(0);
    odometry = new DifferentialDriveOdometry(getHeading2d());
  }

  public void joystickDrive(double forwardSpeed, double rotation, boolean turnInPlace) {
    drive.curvatureDrive(forwardSpeed, rotation, turnInPlace);
  }

  public void rawDrive(double left, double right) {
    drive.tankDrive(left, right, false);
  }

  public void stopDrive() {
    rawDrive(0.0, 0.0);
  }

  public void rawDriveSide(Side side, double speed) {
    (side == Side.LEFT ? leftGroup : rightGroup).set(speed);
  }

  public void velocityDrive(Side side, double sensorUnitsPerDs, double feedforward) {
    List<WPI_TalonFX> motors = side == Side.LEFT ? leftMotors : rightMotors;
    motors.forEach(motor ->
            motor.set(TalonFXControlMode.Velocity, sensorUnitsPerDs, DemandType.ArbitraryFeedForward, feedforward));
  }

  public void resetEncoders() {
    motors.forEach(motor -> motor.setSelectedSensorPosition(0));
  }

  /**
   * Inverts NavX yaw as Odometry takes CCW as positive
   *
   * @return -180..180
   */
  public double getHeading() {
    double heading = -ahrs.getYaw() + yawOffset;
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
  }

  public Rotation2d getHeading2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Zero the gyroscope to specified offset
   *
   * @param heading Desired heading for gyro to read at
   */
  public void zeroHeading(int heading) {
    ahrs.reset();
    yawOffset = heading;
  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    zeroHeading(0);
    odometry.resetPosition(pose2d, getHeading2d());
  }

  public double getLeftPositionMeters() {
    return sensorUnitsToMeters((int) Math.round((frontLeftMotor.getSelectedSensorPosition() + backLeftMotor.getSelectedSensorPosition()) / 2));
  }

  public double getRightPositionMeters() {
    return sensorUnitsToMeters((int) Math.round((frontRightMotor.getSelectedSensorPosition() + backRightMotor.getSelectedSensorPosition()) / 2));
  }

  private double getLeftVelocityMPS() {
    int leftTicksPerDs = (int) Math.round((frontLeftMotor.getSelectedSensorVelocity() + backLeftMotor.getSelectedSensorVelocity()) / 2);
    return sensorUnitsToMeters(leftTicksPerDs * 10);
  }

  private double getRightVelocityMPS() {
    int leftTicksPerDs = (int) Math.round((frontRightMotor.getSelectedSensorVelocity() + backRightMotor.getSelectedSensorVelocity()) / 2);
    return sensorUnitsToMeters(leftTicksPerDs * 10);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMPS(), getRightVelocityMPS());
  }

  public double sensorUnitsToMeters(int sensorUnits) {
    return ((sensorUnits / (double) TALON_UNITS_PER_REV) / GEARING) * WHEEL_CIRCUMFERENCE;
  }

  public double metersToSensorUnits(double meters) {
    return (meters / WHEEL_CIRCUMFERENCE) * GEARING * TALON_UNITS_PER_REV;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (odometryEnabled) {
      odometry.update(getHeading2d(), getLeftPositionMeters(), getRightPositionMeters());
    }

    positionEntry.setString(getPose().toString());
  }

  public void setOdometryEnabled(boolean odometryEnabled) {
    this.odometryEnabled = odometryEnabled;
  }

  public enum Side {
    LEFT, RIGHT
  }
}
