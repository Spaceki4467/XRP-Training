// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.TheDropdownAwakenedLedStyle;
import frc.robot.commands.TheLastDropdownServingIceCreamAddition;
import frc.robot.commands.shapes.SimpleAutoCommand;
import frc.robot.subsystems.XPRLed;
import frc.robot.subsystems.XRPDrivetrain;
import frc.robot.subsystems.XRPServoSubsytem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();
  private final XPRLed m_led = new XPRLed();
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final XRPServoSubsytem m_servo = new XRPServoSubsytem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_autoChooser.addOption("Dropdown", new SimpleAutoCommand(m_drivetrain));
    m_autoChooser.addOption("Dropdown 2.0", new DriveForTimeCommand(m_drivetrain, 1.0, 1.0, 0.0));
    m_autoChooser.addOption("Dropdown 3.0 but good", m_drivetrain.avoidthewallsdropdownfactorycommand());

    m_autoChooser.addOption("PathPlannerDropdownEpicSuperCoolAuto", AutoBuilder.buildAuto("current auto"));

    SmartDashboard.putData("Dropdown", m_autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain, m_controller));
    m_controller.b().onTrue(new TheDropdownAwakenedLedStyle(m_led, true));
    m_controller.x().onTrue(new TheDropdownAwakenedLedStyle(m_led, false));

    m_controller.povLeft().whileTrue(m_drivetrain.turnfordegreesdropdownsometimesfactorycommand(-90));
    m_controller.povRight().whileTrue(m_drivetrain.turnfordegreesdropdownsometimesfactorycommand(90));


    m_controller.a().whileTrue(
            m_servo.setServoPositionFactory(30).withTimeout(0.5).andThen(
                    m_servo.setServoPositionFactory(120).withTimeout(0.5)).repeatedly()
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This is what "command" or task will run when you start autonomous
    // Change it to match whatever shape command you want to run
    return m_autoChooser.getSelected();
  }
}
