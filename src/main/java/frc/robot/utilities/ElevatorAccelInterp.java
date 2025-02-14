package frc.robot.utilities;

import java.util.Arrays;

import frc.robot.Constants.DriveConstants;

public final class ElevatorAccelInterp {
  record HeightEntry(double height, double metersPerSecondSquared) {}

  static final HeightEntry[] heightAccelTable = {
    new HeightEntry(0, 10),
  };

  static final HeightEntry[] heightDecelTable = {
    new HeightEntry(0, 30),
  };

  public static int heightIndex(double height, HeightEntry[] heightTable) {
    assert(height >= 0);
    int index = Arrays.binarySearch(heightTable, new HeightEntry(height, 0), (a,b) -> Double.compare(a.height, b.height));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public static double heightToMaxAcceleration(double height) {
    if (DriveConstants.kLimitSpeedByElevatorHeight) {
      return heightToMaxMetersPerSecondSquared(height, heightAccelTable);
    } else {
      return DriveConstants.kMaxAccelerationMetersPerSecondSquared;
    }
  }

  public static double heightToMaxDeceleration(double height) {
    if (DriveConstants.kLimitSpeedByElevatorHeight) {
      return heightToMaxMetersPerSecondSquared(height, heightDecelTable);
    } else {
      return DriveConstants.kMaxDecelerationMetersPerSecondSquared;
    }
  }

  public static double heightToMaxMetersPerSecondSquared(double height, HeightEntry[] heightTable) {
    int index = heightIndex(height, heightTable);
    if (index == 0) {
      return heightTable[index].metersPerSecondSquared;
    } else if (index == heightTable.length) {
      return heightTable[heightTable.length-1].metersPerSecondSquared;
    } else if (heightTable[index].height == height) {
      return heightTable[index].metersPerSecondSquared;
    }
    // Linear Interpolation
    else {
      double d0 = heightTable[index-1].height;
      double d1 = heightTable[index].height;
      double s0 = heightTable[index-1].metersPerSecondSquared;
      double s1 = heightTable[index].metersPerSecondSquared;
      double scaler = (height-d0) / (d1 - d0);
      double speedRange = s1 - s0;
      return s0 + scaler*speedRange;
    }
  }
}
