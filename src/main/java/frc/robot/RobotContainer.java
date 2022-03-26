// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.BallManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberDirection;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm.ArmDirection;
import frc.robot.util.XboxTrigger;

import static frc.robot.Constants.Joysticks.*;

/** Add your docs here. */
public class RobotContainer {
    private static RobotContainer INSTANCE;

    // Shuffleboard Tabs
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<Command> autoChooser;
    private static final String MAIN_TAB_NAME = "Main";
    private final ShuffleboardTab mainTab = Shuffleboard.getTab(MAIN_TAB_NAME);
    private final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    private NetworkTableEntry autoTimes;
    // Subsystems
    private final DriveTrain driveTrain = new DriveTrain();
    private final Arm arm = new Arm();
    private final BallManipulator ballManipulator = new BallManipulator();
    private final Climber climber = new Climber();

    // Controllers
    private final Joystick joystick = new Joystick(LOGITECH_JOYSTICK);
    private final XboxController xboxController = new XboxController(XBOX_CONTROLLER);

    // Commands
    /* Drive commands */
    private final DriveJoystickCommand driveJoystickCommand = new DriveJoystickCommand(driveTrain, joystick);
    /* Automatic motion profile arm commands */
    private final ArmMoveToCommand armAutoUpCommand = new ArmMoveToCommand(arm, ArmPosition.UP);
    private final ArmMoveToCommand armAutoClimbPositionCommand = new ArmMoveToCommand(arm, ArmPosition.CLIMB);
    private final ArmMoveToCommand armAutoDownCommand = new ArmMoveToCommand(arm, ArmPosition.DOWN);
    /* Arm manual commands, using percent output, in case encoder fails */
    private final ArmManualMoveCommand armManualUpCommand = new ArmManualMoveCommand(arm, ArmDirection.UP);
    private final ArmManualMoveCommand armManualDownCommand = new ArmManualMoveCommand(arm, ArmDirection.DOWN);
    /* Arm manual commands, using PID, that change setpoint */
    private final ArmManualHoldCommand armManualHoldUpCommand = new ArmManualHoldCommand(arm, ArmDirection.UP);
    private final ArmManualHoldCommand armManualHoldDownCommand = new ArmManualHoldCommand(arm, ArmDirection.DOWN);
    /* Ball commands */
    private final IntakeCommand intakeCommand = new IntakeCommand(ballManipulator);
    private final ShootCommand shootCommand = new ShootCommand(ballManipulator);
    /* Climber commands */
    private final ClimberManualMoveCommand climberManualUpCommand =
            new ClimberManualMoveCommand(climber, ClimberDirection.UP);
    private final ClimberManualMoveCommand climberManualDownCommand =
            new ClimberManualMoveCommand(climber, ClimberDirection.DOWN);
    private final ClimberManualMoveFastCommand climberManualUpFastCommand =
            new ClimberManualMoveFastCommand(climber, ClimberDirection.UP);
    private final ClimberManualMoveFastCommand climberManualDownFastCommand =
            new ClimberManualMoveFastCommand(climber, ClimberDirection.DOWN);


    public RobotContainer() {
        INSTANCE = this;
        // Configure controller bindings
        configureButtonBindings();
        // Set default commands
        driveTrain.setDefaultCommand(driveJoystickCommand);
        // Shuffleboard
        setupAutonomousTab();
        autoTimes = debugTab.add("Auto Path Times", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(5, 2)
                .getEntry();
    }

    public void addAutoTime(String path, double seconds) {
        String current = autoTimes.getString("");
        boolean isFirst = current.equals("");
        autoTimes.setString(current + (isFirst ? "" : "; ") + String.format("%s @ %.2fs", path, seconds));
    }

    public static RobotContainer getInstance() {
        return INSTANCE;
    }

    private void configureButtonBindings() {
        /* REFERENCE:
         When Pressed/Active = schedules on press, does not schedule until pressed again
         When Held/Active Once = Schedules on press, cancels on depress
        */

        /* LOGITECH CONTROLLER BINDINGS */

        /* XBOX CONTROLLER BINDINGS */
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

        JoystickButton startButton = new JoystickButton(xboxController, XboxController.Button.kStart.value);
        startButton.whenHeld(armAutoClimbPositionCommand);
        JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
        yButton.whenHeld(climberManualUpCommand);
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        bButton.whenHeld(climberManualUpFastCommand);
        JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
        xButton.whenHeld(climberManualDownFastCommand);
        JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        aButton.whenHeld(climberManualDownCommand);
    }

    private void setupAutonomousTab() {
        autoChooser = new SendableChooser<>();
        SendableRegistry.setName(autoChooser, "Autonomous Command");
        // Setup autonomous command options
        autoChooser.setDefaultOption("Nothing", null);

        autoTab.add(autoChooser).withPosition(0, 0).withSize(2, 1);
    }

    public Command getAutonomousCommand() {
        return (autoChooser != null ? autoChooser.getSelected() : null);
    }

    public void teleopInit() {
        // Switch to main Shuffleboard tab at start of teleop
        Shuffleboard.selectTab(MAIN_TAB_NAME);
    }
}
