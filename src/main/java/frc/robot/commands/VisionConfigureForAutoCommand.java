package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

public class VisionConfigureForAutoCommand extends InstantCommand {
    public VisionConfigureForAutoCommand(Vision vision) {
        super(() -> {
            //vision.setDriverMode(false);
            vision.setPipeline();
        }, vision);
    }
}
