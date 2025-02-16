// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//This file is supposed to identify a color and flash it when prompted.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LightSubsystem extends SubsystemBase {
  private final Spark m_blinkin;

  public LightSubsystem() {
    m_blinkin = new Spark(LightConstants.kBlinkinPWMInput);
  }

  public enum Color {
    RED(0.61),
    ORANGE(0.63),
    YELLOW(0.69),
    GREEN(0.77),
    BLUE(0.87),
    PURPLE(0.89),
    WHITE(0.93),
    PINK(0.57),
    OFF(0.99);

    private final double m_colorValue;
    private Color(double colorValue) {
      this.m_colorValue = colorValue;
    }
  }

  public void setColor(Color color) {
    m_blinkin.set(color.m_colorValue);
  }
}
