package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPServoSubsytem extends SubsystemBase {
    private final XRPServo m_servo = new XRPServo(4);

    public void setServoPosition(double angle) {
        m_servo.setAngle(angle);
    }
    public Command setServoPositionFactory(double angle) {
        return run(() -> setServoPosition(angle));
    }
}
