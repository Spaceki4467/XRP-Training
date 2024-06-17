package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

public class DriveForTimeCommand extends Command {

    private final XRPDrivetrain m_drivetrain;
    private final Timer m_timer;
    private final double m_xSpeed;
    private final double m_zRotation;
    private final double m_time;

    public DriveForTimeCommand(XRPDrivetrain drivetrain, double time, double xSpeed, double zRotation){

        m_drivetrain = drivetrain;
        m_timer = new Timer();
        m_xSpeed = xSpeed;
        m_zRotation = zRotation;
        m_time = time;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

    }
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(m_xSpeed, m_zRotation);
    }
    @Override
    public void end(boolean interruped){
        m_timer.stop();

    }
    @Override
    public boolean isFinished() {
        return m_timer.get() >= m_time;
    }
}
