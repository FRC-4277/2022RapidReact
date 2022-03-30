// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.DriveTrain.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomSimField;

import java.util.List;
import java.util.stream.Collectors;

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

  // Simulation
  private final CustomSimField simField;
  private DifferentialDrivetrainSim simulator;
  private List<TalonFXSimCollection> leftDriveSims;
  private List<TalonFXSimCollection> rightDriveSims;
  private SimDouble simAngle;

  // Shuffleboard
  private final ShuffleboardTab autoTab;
  private final NetworkTableEntry positionEntry;

  // State variables
  private boolean odometryEnabled = true;

  public DriveTrain(CustomSimField simField, ShuffleboardTab autoTab) {
    this.simField = simField;
    this.autoTab = autoTab;

    positionEntry = autoTab.add("Position", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(2, 0)
      .withSize(4, 1)
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

    // Simulation
    if (RobotBase.isSimulation()) {
      simulator = new DifferentialDrivetrainSim(
        // 1.5 and 0.3 are guesses for angular gains (does not matter as it's a simulation)
        LinearSystemId.identifyDrivetrainSystem(KV_LINEAR, KA_LINEAR, 1.5, 0.3),
        DCMotor.getFalcon500(2),
        GEARING,
        TRACK_WIDTH_M,
        WHEEL_DIAMETER_M / 2,
        VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
      );

      leftDriveSims = leftMotors.stream().map(TalonFX::getSimCollection).collect(Collectors.toList());
      rightDriveSims = rightMotors.stream().map(TalonFX::getSimCollection).collect(Collectors.toList());

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }
  }

  public void joystickDrive(double forwardSpeed, double rotation, boolean turnInPlace) {
    drive.curvatureDrive(forwardSpeed, rotation, turnInPlace);
  }

  public void rawDrive(double left, double right) {
    drive.tankDrive(left, right, false);
    System.out.println();
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
    //System.out.println("getHeading() returns" + heading);
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
    //System.out.println("FRONT LEFT TALON FX UNITS: " + frontLeftMotor.getSelectedSensorPosition());
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

  public int metersToSensorUnits(double meters) {
    return (int) Math.round((meters / WHEEL_CIRCUMFERENCE) * GEARING * TALON_UNITS_PER_REV);
  }

  public static double convertPercentToVelocity(double percentOutput) {
    return MOTOR_FEEDFORWARD.maxAchievableVelocity(MAX_BATTERY_V * percentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (odometryEnabled || RobotBase.isSimulation()) {
      /*System.out.println("ODOMETRY UPDATE HEADING: " + getHeading2d());
      System.out.println("ODOMETRY UPDATE METERS L: " + getLeftPositionMeters());
      System.out.println("ODOMETRY UPDATE METERS R: " + getRightPositionMeters());*/
      odometry.update(getHeading2d(), getLeftPositionMeters(), getRightPositionMeters());
    }

    // Display position
    simField.setRobotPosition(getPose());
    positionEntry.setString(getPose().toString());
  }

  @Override
  public void simulationPeriodic() {
    // Update physics sim
    simulator.setInputs(leftDriveSims.get(0).getMotorOutputLeadVoltage(),
            -rightDriveSims.get(0).getMotorOutputLeadVoltage());
    //System.out.println("L V: " + leftDriveSims.get(0).getMotorOutputLeadVoltage());
    //System.out.println("R V: " + -rightDriveSims.get(0).getMotorOutputLeadVoltage());
    simulator.update(0.02);

    // Update Talon FXs
    //System.out.println("L sim meters: " + simulator.getLeftPositionMeters());
    //System.out.println("L sim sensor u: " + metersToSensorUnits(simulator.getLeftPositionMeters()));
    leftDriveSims.forEach(leftDriveSim -> {
      leftDriveSim.setIntegratedSensorRawPosition(metersToSensorUnits(simulator.getLeftPositionMeters()));
      leftDriveSim.setIntegratedSensorVelocity(
              (int) Math.round((double) metersToSensorUnits(simulator.getLeftVelocityMetersPerSecond()) / 10));
    });
    rightDriveSims.forEach(rightDriveSim -> {
      rightDriveSim.setIntegratedSensorRawPosition(-metersToSensorUnits(simulator.getRightPositionMeters()));
      rightDriveSim.setIntegratedSensorVelocity(
              (int) -Math.round((double) metersToSensorUnits(simulator.getRightVelocityMetersPerSecond()) / 10));
    });

    // Update NavX
    // NavX expects clockwise positive, but sim outputs clockwise negative
    //System.out.println("SIM ANGLE SET TO " + Math.IEEEremainder(-simulator.getHeading().getDegrees(), 360));
    simAngle.set(Math.IEEEremainder(-simulator.getHeading().getDegrees(), 360));
  }

  public void setOdometryEnabled(boolean odometryEnabled) {
    this.odometryEnabled = odometryEnabled;
  }

  public enum Side {
    LEFT, RIGHT
  }
}
