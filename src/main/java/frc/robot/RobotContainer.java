// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.Joysticks.*;

/** Add your docs here. */
public class RobotContainer {
    private DriveTrain driveTrain = new DriveTrain();
    private Joystick joystick = new Joystick(LOGITECH_JOYSTICK);

    private JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand(driveTrain, joystick);

    public RobotContainer() {
        configureButtonBindings();

        driveTrain.setDefaultCommand(joystickDriveCommand);
    }

    public void configureButtonBindings() {
  
    }
}
