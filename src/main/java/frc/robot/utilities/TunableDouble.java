package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class TunableDouble {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("tuning");
  private final NetworkTableEntry m_dashboardEntry;
  private final double m_defaultValue;
  private double m_currentValue;
  
  public TunableDouble(String key, double defaultValue) {
    m_defaultValue = defaultValue;
    if (Constants.kEnableTuning) {
      m_dashboardEntry = table.getEntry(key);
      m_dashboardEntry.setNumber(defaultValue);
    } else {
      m_dashboardEntry = null;
    }
  }

  public double get() {
    if (Constants.kEnableTuning) {
      update();
      return m_currentValue;
    } else {
      return m_defaultValue;
    }
  }

  private void update() {
    m_currentValue = m_dashboardEntry.getDouble(m_defaultValue);
  }

  private double peek() {
    return m_dashboardEntry.getDouble(m_defaultValue);
  }

  public boolean hasChanged() {
    if (Constants.kEnableTuning) {
      double newValue = peek();
      return newValue != m_currentValue;
    } else {
      return false;
    }
  }
}