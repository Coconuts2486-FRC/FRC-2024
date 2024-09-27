package frc.robot.subsystems.pdh;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class PDH extends VirtualSubsystem {

  private static final int PDH_CAN_ID = 1;
  private static final int NUM_PDH_CHANNELS = 24;

  PowerDistribution m_pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  public PDH() {}

  public void periodic() {

    Logger.recordOutput("PDH/Voltage", m_pdh.getVoltage());
    Logger.recordOutput("PDH/TotalCurrent", m_pdh.getTotalCurrent());
    for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
      Logger.recordOutput(
          "PDH/CurrentChannel" + String.valueOf(channel), m_pdh.getCurrent(channel));
    }
  }
}
