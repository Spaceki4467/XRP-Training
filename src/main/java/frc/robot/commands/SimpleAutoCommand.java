package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.XRPDrivetrain;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand(XRPDrivetrain drivetrain) {
        addCommands(new DriveForTimeCommand(drivetrain, 2.0, 0.0, 1.0));
    }
}
