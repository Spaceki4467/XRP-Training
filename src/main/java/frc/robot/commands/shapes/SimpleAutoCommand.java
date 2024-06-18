package frc.robot.commands.shapes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.subsystems.XRPDrivetrain;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand(XRPDrivetrain drivetrain) {
        addCommands(
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.5),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.3),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, -0.7),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.9),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.1),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, -0.2),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, -0.5),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.6)
        );
    }
}
