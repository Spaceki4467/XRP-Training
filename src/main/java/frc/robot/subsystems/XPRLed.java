package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XPRLed extends SubsystemBase {
    private final XRPOnBoardIO m_io = new XRPOnBoardIO();

    public XPRLed() {
        m_io.setLed(false);
    }
    public void setLedOn(){
        m_io.setLed(true);
    }
    public void setLedOff(){
        m_io.setLed(false);
    }
}
