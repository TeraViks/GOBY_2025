package frc.robot.utilities;

import java.util.Arrays;

import frc.robot.Constants.DriveConstants;

public final class ElevatorSpeedInterp {
  record HeightEntry(double height, double speed) {}

  static final HeightEntry[] heightTable = {
    new HeightEntry(0, 5),
  };

  public static double getMaxSpeed() {
    return heightTable[heightTable.length-1].speed;
  }

  public static int heightIndex(double height) {
    assert(height >= 0);
    int index = Arrays.binarySearch(heightTable, new HeightEntry(height, 0), (a,b) -> Double.compare(a.height, b.height));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public static double heightToMaxSpeed(double height) {
    if (DriveConstants.kLimitSpeedByElevatorHeight) {
      return getMaxSpeed();
    }
    int index = heightIndex(height);
    if (index == 0) {
      return heightTable[index].speed;
    } else if (index == heightTable.length) {
      return heightTable[heightTable.length-1].speed;
    } else if (heightTable[index].height == height) {
      return heightTable[index].speed;
    }
    // Linear Interpolation
    else {
      double d0 = heightTable[index-1].height;
      double d1 = heightTable[index].height;
      double s0 = heightTable[index-1].speed;
      double s1 = heightTable[index].speed;
      double scaler = (height-d0) / (d1 - d0);
      double speedRange = s1 - s0;
      return s0 + scaler*speedRange;
    }
  }
}
