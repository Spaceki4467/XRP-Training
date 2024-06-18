package frc.robot.commands;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XPRLed;
import frc.robot.subsystems.XRPServoSubsytem;

public class TheLastDropdownServingIceCreamAddition extends Command {

    private final XRPServoSubsytem m_servo;
    private final double m_angle;

    public TheLastDropdownServingIceCreamAddition(XRPServoSubsytem servo, double angle){
        m_servo = servo;
        m_angle = angle;
        addRequirements(m_servo);
    }

    @Override
    public void initialize() {
        m_servo.setServoPosition(m_angle);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}