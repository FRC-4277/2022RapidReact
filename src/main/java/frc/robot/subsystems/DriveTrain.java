// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.DriveTrain.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX frontLeftMotor = new WPI_TalonFX(FRONT_LEFT);
  private WPI_TalonFX frontRightMotor = new WPI_TalonFX(FRONT_RIGHT);
  private WPI_TalonFX backLeftMotor = new WPI_TalonFX(BACK_LEFT);
  private WPI_TalonFX backRightMotor = new WPI_TalonFX(BACK_RIGHT);

  private MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  
//"space apart for clarity", Andrew Tran.



  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //this code resets the motors everytime it runs.
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    frontLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    frontRightMotor.setInverted(TalonFXInvertType.Clockwise);
    backLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    backRightMotor.setInverted(TalonFXInvertType.Clockwise);

  }

  public void joystickDrive(double forwardSpeed, double rotation) {
    drive.arcadeDrive(forwardSpeed, rotation);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
