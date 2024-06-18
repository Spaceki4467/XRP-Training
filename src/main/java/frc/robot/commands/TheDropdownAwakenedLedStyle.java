package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XPRLed;

public class TheDropdownAwakenedLedStyle extends Command {

    private final XPRLed m_led;
    private final boolean m_ledOn;

    public TheDropdownAwakenedLedStyle(XPRLed led, boolean ledOn){
        m_led = led;
        m_ledOn = ledOn;

        addRequirements(m_led);
    }

    @Override
    public void initialize() {
        if (m_ledOn) {
            m_led.setLedOn();
        } else {
            m_led.setLedOff();
        }
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}

