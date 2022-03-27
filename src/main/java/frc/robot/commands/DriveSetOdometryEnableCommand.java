// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveSetOdometryEnableCommand extends InstantCommand {

    public DriveSetOdometryEnableCommand(DriveTrain driveTrain, boolean enabled) {
        super(() -> driveTrain.setOdometryEnabled(enabled), driveTrain);
    }
}
