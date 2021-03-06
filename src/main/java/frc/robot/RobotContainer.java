// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.CargoManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberDirection;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
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
    private final NetworkTableEntry autoTimes, remainingTime, climbWarning;
    {
        autoTimes = debugTab.add("Auto Path Times", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(5, 2)
                .getEntry();
    }

    // Simulator
    private final CustomSimField simField = new CustomSimField();

    // Subsystems
    private final DriveTrain driveTrain = new DriveTrain(simField, autoTab);
    private final Arm arm = new Arm(simField, mainTab);
    private final CargoManipulator cargoManipulator = new CargoManipulator(simField);
    private final Climber climber = new Climber(mainTab);
    private final Vision vision = new Vision(mainTab);

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
    private final CargoIntakeCommand cargoIntakeCommand = new CargoIntakeCommand(cargoManipulator);
    private final CargoShootCommand cargoShootCommand = new CargoShootCommand(cargoManipulator);
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
        remainingTime = mainTab.add("Remaining Teleop Time", -1)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 2)
                .withSize(2, 1)
                .getEntry();
        climbWarning = mainTab.add("Climb Warning", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(7, 2)
                .getEntry();
        // Disable LiveWindow as we don't need it
        LiveWindow.disableAllTelemetry();
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
        rightBumper.whenHeld(cargoIntakeCommand);
        Trigger rightTrigger = new XboxTrigger(xboxController, false);
        rightTrigger.whileActiveOnce(cargoShootCommand);

        JoystickButton startButton = new JoystickButton(xboxController, XboxController.Button.kStart.value);
        startButton.whenPressed(armAutoClimbPositionCommand);
        JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
        yButton.whenHeld(climberManualUpCommand);
        JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
        bButton.whenHeld(climberManualUpFastCommand);
        JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
        xButton.whenHeld(climberManualDownFastCommand);
        JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        aButton.whenHeld(climberManualDownCommand);

        JoystickButton backButton = new JoystickButton(xboxController, XboxController.Button.kBack.value);
        backButton.whenHeld(new ParallelCommandGroup(
            new ArmMoveToCommand(arm, ArmPosition.DOWN),
            new CargoIntakeCommand(cargoManipulator)
        ));
        backButton.whenReleased(new ParallelCommandGroup(
            new ArmMoveToCommand(arm, ArmPosition.UP),
            new ProxyScheduleCommand(new CargoIntakeCommand(cargoManipulator))
        ));
    }

    private void setupAutonomousTab() {
        autoChooser = new SendableChooser<>();
        SendableRegistry.setName(autoChooser, "Autonomous Command");
        // Setup autonomous command options
        autoChooser.setDefaultOption("Nothing", null);
        autoChooser.addOption("Arm Down", new ArmFirstDownCommand(arm));
        autoChooser.addOption("Move Backwards", new MoveOnlyAuto0(driveTrain, arm));
        autoChooser.addOption("Shoot & Move Backwards", new ShootMoveAuto1(driveTrain, cargoManipulator, arm));
        autoChooser.addOption("Two Ball (A)", new TwoBallAuto2(cargoManipulator, driveTrain, arm, Cargo.A, vision));
        autoChooser.addOption("Two Ball (B)", new TwoBallAuto2(cargoManipulator, driveTrain, arm, Cargo.B, vision));
        autoChooser.addOption("Two Ball (D)", new TwoBallAuto2(cargoManipulator, driveTrain, arm, Cargo.D, vision));
        autoChooser.addOption("Three Ball (A)", new ThreeBallAuto3(driveTrain, cargoManipulator, arm));
        autoChooser.addOption("Four Ball (A)", new FourBallAuto4(driveTrain, cargoManipulator, arm, vision));

        autoTab.add(autoChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
        autoTab.add(simField.getField2d())
                .withWidget(BuiltInWidgets.kField)
                .withPosition(0, 1)
                .withSize(7, 4);
    }

    public Command getAutonomousCommand() {
        return (autoChooser != null ? autoChooser.getSelected() : null);
    }

    public void teleopInit() {
        // Switch to main Shuffleboard tab at start of teleop
        Shuffleboard.selectTab(MAIN_TAB_NAME);
        // Turn off odometry in teleop
        driveTrain.setOdometryEnabled(false);
        // Set vision to driver mode
        vision.setDriverMode(true);
    }

    public void teleopPeriodic() {
        double timeRemaining = DriverStation.getMatchTime();
        remainingTime.setNumber(timeRemaining);
        climbWarning.setBoolean(timeRemaining <= 35.0);
    }

    public CustomSimField getSimField() {
        return simField;
    }
}
