// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmAutoMoveCommand;
import frc.robot.commands.ArmManualHoldCommand;
import frc.robot.commands.ArmManualMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallManipulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm.ArmDirection;
import frc.robot.util.XboxTrigger;

import static frc.robot.Constants.Joysticks.*;

/** Add your docs here. */
public class RobotContainer {
    private DriveTrain driveTrain = new DriveTrain();
    private Arm arm = new Arm();
    private BallManipulator ballManipulator = new BallManipulator();
    
    private Joystick joystick = new Joystick(LOGITECH_JOYSTICK);
    private XboxController xboxController = new XboxController(XBOX_CONTROLLER);

    private JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand(driveTrain, joystick);
    
    private ArmAutoMoveCommand armAutoUpCommand = new ArmAutoMoveCommand(arm, ArmDirection.UP);
    private ArmAutoMoveCommand armAutoDownCommand = new ArmAutoMoveCommand(arm, ArmDirection.DOWN);
    private ArmManualMoveCommand armManualUpCommand = new ArmManualMoveCommand(arm, ArmDirection.UP);
    private ArmManualMoveCommand armManualDownCommand = new ArmManualMoveCommand(arm, ArmDirection.DOWN);
    private ArmManualHoldCommand armManualHoldUpCommand = new ArmManualHoldCommand(arm, ArmDirection.UP);
    private ArmManualHoldCommand armManualHoldDownCommand = new ArmManualHoldCommand(arm, ArmDirection.DOWN);

    private IntakeCommand intakeCommand = new IntakeCommand(ballManipulator);
    private ShootCommand shootCommand = new ShootCommand(ballManipulator);

    public RobotContainer() {
        configureButtonBindings();

        
        System.out.println("Default Command Set");
        driveTrain.setDefaultCommand(joystickDriveCommand);
        System.out.println("Done!!!");
    }

    public void configureButtonBindings() {
        // When Pressed/Active = schedules on press, does not schedule until pressed again
        // When Held/Active Once = Schedules on press, cancels on depress
        JoystickButton leftBumper = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
        leftBumper.whenPressed(armAutoUpCommand);
        Trigger leftTrigger = new XboxTrigger(xboxController, true);
        leftTrigger.whenActive(armAutoDownCommand);

        POVButton upPOV = new POVButton(xboxController, 0);
        upPOV.whenHeld(armManualUpCommand);
        POVButton downPOV = new POVButton(xboxController, 180);
        downPOV.whenHeld(armManualDownCommand);
        POVButton leftPOV = new POVButton(xboxController, 270);
        leftPOV.whenPressed(armManualHoldDownCommand);
        POVButton rightPOV = new POVButton(xboxController, 90);
        rightPOV.whenPressed(armManualHoldUpCommand);

        JoystickButton rightBumper = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
        rightBumper.whenHeld(intakeCommand);
        Trigger rightTrigger = new XboxTrigger(xboxController, false);
        rightTrigger.whileActiveOnce(shootCommand);
    }
}
