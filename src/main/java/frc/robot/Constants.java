// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CameraSubsystem.CameraConfig;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.Segment;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TrapezoidalConstraint;

public final class Constants {
  // Set to true during actual competition. May disable diagnostics, sanity/safety checks, etc.
  // in order to eliminate failure modes that should only happen during testing.
  public static final boolean kCompeting = false;

  // Scheduling quantum in seconds.
  public static final double kDt = 0.02;

  public static final PersistMode kPersistMode = PersistMode.kNoPersistParameters;

  // Tunable constants are disabled unless this is set to true.
  // Intended to remain false in committed code.
  public static final boolean kEnableTuning = false;

  public static final class DriveConstants {
    public static final boolean kSquareInputs = false;

    public static final double kTrackWidth = 0.629; // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.629; // Distance between front and back wheels on robot

    public static final Vector<N3> stateStdDeviations =
      VecBuilder.fill(0.005, 0.005, Math.toRadians(1.0));
    public static final Vector<N3> visionStdDeviations =
      VecBuilder.fill(0.050, 0.050, Math.toRadians(5.0));

    public static final boolean kGyroReversed = false;

    // If the aggregate velocity of the swerve modules is beneath this threshold, the robot is considered to be at rest,
    // in which case housekeeping such as re-synchronizing the turning encoders may take place.
    public static final double kAggregateSpeedThresholdMetersPerSecond = 0.01;

    // Note that SwerveModuleConstants.kMaxSpeedMedersPerSecond may saturate if this is set too high, in combination with
    // SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond.
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxDecelerationMetersPerSecondSquared = 30.0;

    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3.0 * Math.PI;
    public static final double kMaxAngularDecelerationRadiansPerSecondSquared = 9.0 * Math.PI;

    public static final boolean kLimitSpeedByElevatorHeight = false;

    public static final TrapezoidalConstraint kAngularVelocityProfile = new TrapezoidalConstraint(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      () -> kMaxAngularAccelerationRadiansPerSecondSquared,
      () -> kMaxAngularDecelerationRadiansPerSecondSquared
    );
  }

  public static final class SwerveModuleConstants {
    public static final ClosedLoopSlot kAutoPIDFSlotID = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kTeleopPIDFSlotID = ClosedLoopSlot.kSlot1;
    public static final ClosedLoopSlot kDefaultPIDFSlotID = kAutoPIDFSlotID;

    private static final double kUnsigned_0To1 = 1.0;
    public static final double kAbsoluteSensorDiscontinuityPoint = kUnsigned_0To1;

    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond * 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = Math.max(
      DriveConstants.kMaxAccelerationMetersPerSecondSquared,
      DriveConstants.kMaxDecelerationMetersPerSecondSquared
    );

    public static final double kWheelDiameterMeters = 0.09525;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurningGearRatio = 150.0 / 7.0;

    // The native position units are motor rotations, but we want meters.
    public static final double kDrivePositionConversionFactor =
      (SwerveModuleConstants.kWheelDiameterMeters * Math.PI)
      / SwerveModuleConstants.kDriveGearRatio;
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM),
    // but we want meters per second.
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor
      / 60.0 /* s */;

    // The native position units are motor rotations, but we want radians.
    public static final double kTurningPositionConversionFactor =
      Units.rotationsToRadians(1.0 / SwerveModuleConstants.kTurningGearRatio);
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM), but we want radians per second.
    public static final double kTurningVelocityConversionFactor =
      kTurningPositionConversionFactor
      / 60.0 /* s */;

    // The NEO's relative encoder can return spurious results during initialization, and there is
    // no synchronous API for setting the encoder value. This presents a conundrum regarding how to
    // get valid/stable output early on. Our heuristic strategy is to set the encoder to zero, and
    // keep reading the encoder value until it is acceptably close to zero. This value can be quite
    // low, but there is the hypothetical danger of encoder jitter causing an infinite loop as the
    // tolerance approches zero.
    public static final double kTurningEncoderStabilizeToleranceRadians =
      Units.degreesToRadians(1.0);

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kTurningMotorCurrentLimit = 20;

    public static final double kDriveMotorRampRate = 0.25;
    public static final double kTurningMotorRampRate = 0.25;

    public static final long kValueCacheTtlMicroseconds = 15;

    // Tune via SwerveModule.tunable*DrivePIDF rather than iteratively modifying these fields.
    public static final SparkUtil.PIDFSlot kAutoDrivePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0, 0.25),
      kAutoPIDFSlotID
    );
    public static final SparkUtil.PIDFSlot kTeleopDrivePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0, 0.25),
      kTeleopPIDFSlotID
    );
    public static final SparkUtil.Config kFrontLeftDriveMotorConfig = new SparkUtil.Config(
      kDriveMotorCurrentLimit,
      kDriveMotorRampRate,
      true,
      kDriveVelocityConversionFactor,
      kDrivePositionConversionFactor,
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared,
      new ArrayList<>() {{
        add(kAutoDrivePIDFSlot);
        add(kTeleopDrivePIDFSlot);
      }}
    );
    public static final SparkUtil.Config kRearLeftDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);
    public static final SparkUtil.Config kRearRightDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);
    public static final SparkUtil.Config kFrontRightDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);

    // Tune via SwerveModule.tunable*TurningPIDF rather than iteratively modifying these fields.
    public static final SparkUtil.PIDFSlot kAutoTurningPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0),
      kAutoPIDFSlotID
    );
    public static final SparkUtil.PIDFSlot kTeleopTurningPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0),
      kTeleopPIDFSlotID
    );
    public static final SparkUtil.Config kFrontLeftTurningMotorConfig = new SparkUtil.Config(
      kTurningMotorCurrentLimit,
      kTurningMotorRampRate,
      true,
      kTurningVelocityConversionFactor,
      kTurningPositionConversionFactor,
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared,
      new ArrayList<>() {{
        add(kAutoTurningPIDFSlot);
        add(kTeleopTurningPIDFSlot);
      }}
    );
    public static final SparkUtil.Config kRearLeftTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);
    public static final SparkUtil.Config kRearRightTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);
    public static final SparkUtil.Config kFrontRightTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);

    public static final SwerveModule.Config kFrontLeftSwerveConfig = new SwerveModule.Config(
      1, 5, 9,
      kFrontLeftDriveMotorConfig, kFrontLeftTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.266357))
    );
    public static final SwerveModule.Config kRearLeftSwerveConfig = new SwerveModule.Config(
      2, 6, 10,
      kRearLeftDriveMotorConfig, kRearLeftTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.281982))
    );
    public static final SwerveModule.Config kRearRightSwerveConfig = new SwerveModule.Config(
      3, 7, 11,
      kRearRightDriveMotorConfig, kRearRightTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.476562))
    );
    public static final SwerveModule.Config kFrontRightSwerveConfig = new SwerveModule.Config(
      4, 8, 12,
      kFrontRightDriveMotorConfig, kFrontRightTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.367920))
    );
  }

  public static final class HandlerConstants {
    public static final int kMotorID = 17;
    public static final double kIntakeSpeedCoral = 0.25; //TODO: all of these constants are placeholders
    public static final double kEjectSpeedCoral = -0.25;
    public static final double kIntakeSpeedAlgae = -0.4; //TODO: Make sure the speeds for the algae and coral are reversed
    public static final double kEjectSpeedAlgae = 0.1;
    public static final double kTestSpeed = 0.0;

    public static final double kEjectDelaySeconds = 0.25;
    public static final int kHoldingCurrentLimit = 15;
    public static final int kNormalCurrentLimit = 40;
    
    //TODO: theese constants are placeholders
    public static final int kAlgaeSensorInput = 0;
    public static final int kBackSensorInput = 3;
    public static final int kFrontSensorInput = 2;
    public static final int kDistanceSensorInput = 1;

    public static final double kDebounceTime = 0.0;
    public static final double kAlgaeSensorProxThreshold = 0.02;
    public static final double kBackSensorProxThreshold = 0.02;
    public static final double kFrontSensorProxThreshold = 0.03;
    public static final double kDistanceSensorProxThreshold = 0.08;

    public static final ClosedLoopSlot kPIDFSlotVelocity = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kPIDFSlotPosition = ClosedLoopSlot.kSlot1;
    public static final PIDF kMotorPIDFVel = new PIDF(2.5, 0.0, 50.0, 1.55);
    public static final PIDF kMotorPIDFPos = new PIDF(2.5, 0.0, 50.0, 1.55);

    public static final SparkUtil.Config kmotorConfig = new SparkUtil.Config(
      kNormalCurrentLimit,
      0.0, 
      true,
      1.083e-4, 
      0.0065, // 4 in. wheel and 2.5^3 gear reduction
      1.0, // TODO: Configure.
      2.0, // TODO: Configure.
      new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(
          kMotorPIDFVel,
          kPIDFSlotVelocity
        ));
        add(new SparkUtil.PIDFSlot(
        kMotorPIDFPos,
        kPIDFSlotPosition
        ));
      }}
    );
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kMaxRadPerSec = DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kBack = 7;
    public static final int kStart = 8;
    public static final int kLeftJoy = 9;
    public static final int kRightJoy = 10;

    public static final int kLeftJoyXAxis = 0;
    public static final int kLeftJoyYAxis = 1;
    public static final int kLeftTriggerAxis = 2;
    public static final int kRightTriggerAxis = 3;
    public static final int kRightJoyXAxis = 4;
    public static final int kRightJoyYAxis = 5;

    public static final int kZeroGyro = kStart;
    public static final int kFaceReefButton = kY;
    public static final int kFaceCoralStationButton = kX;
    public static final int kFaceProcessorButton = kA;
    public static final int kASideButton = kLeftBumper;
    public static final int kBSideButton = kRightBumper;
    public static final int kLevel1Button = kA;
    public static final int kLevel2Button = kBack;
    public static final int kLevel3Button = kStart;
    public static final int kIntakeCoralButton = kX;
    public static final int kIntakeALgaePOV = 0;
    public static final int kEjectButton = kY;
    public static final int kHighAlgaePOV = 90;
    public static final int kLowAlgaePOV = 270;
    public static final int kAlgaeProcessorPOV = 180;
    public static final int kExtendClimberAxis = kLeftTriggerAxis;
    public static final int kRetractClimberAxis = kRightTriggerAxis;
    public static final int kCoralIntake2 = 0;
    public static final int kCoralIntake5 = 90;
    public static final int kCoralIntake8 = 180;

    public static final double kTriggerAcuationValue = 0.5;
    
    public static final double kDebounceSeconds = 0.01;

    public static final double kJoystickDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants kTranslationHolonomicPID= new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationHolonomicPID= new PIDConstants(5.0, 0.0, 0.0);

    public static final double kDriveBaseRadius = 0.4;
    public static final double kBumperToRobotCenter = 0.381; //TODO: add bumper width

    public static final double kStingerCenterOffset = 0.0; // TODO: Tune.

    public static final double kStationWallOffset = 0.1524; // TODO: Tune.
    public static final double kStationSlotSpacing = 0.202;

    public static final double kReefWallOffset = 0.1524; // TODO: Tune.
    public static final double kReefBranchOffset = 0.164;

    public static final double kProcessorWallOffset = 0.1524; // TODO: Tune.
  }

  public static final class CameraConstants {
    public static final boolean kEnable = false;
    public static final List<CameraConfig> kCameraConfigs = List.of(
      new CameraConfig(
        "reefCamera",
        new Transform3d(
          0.279292, 0.137998, 0.317346,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(21.5))),
        false
      ),
      new CameraConfig(
        "coralStationCamera",
        new Transform3d(
          0.283631, 0.130006, 0.935310,
          new Rotation3d(0.0, Units.degreesToRadians(-22.5), Units.degreesToRadians(9.5))),
        false
      )
    );
  }

  public static final class DriveCommandConstants {
    public static final double kDefaultTranslationPositionToleranceMeters = 0.02;
    public static final double kDefaultTranslationVelocityToleranceMetersPerSecond = 0.01;
    public static final double kDefaultAnglePositionToleranceRadians = Units.degreesToRadians(2.0);
    public static final double kDefaultAngleVelocityToleranceRadiansPerSecond = Units.degreesToRadians(1.0);

    public static final PIDF kTranslatingPIDF = new PIDF(0.0, 0.0, 0.0, 0.0); //TODO: Tune.
    public static final PIDF kTurningPIDF = new PIDF(3.0, 0.0, 0.0, 0.2); //TODO: Update the PIDF values (copied from AC/DC)
  }

  public static final class FieldConstants {
    private static final AprilTagFieldLayout loadTransformedAprilTagFieldLayout() {
      final AprilTagFields rawLayoutFile = AprilTagFields.k2025ReefscapeWelded;
      final AprilTagFieldLayout rawLayout = AprilTagFieldLayout.loadField(rawLayoutFile);
      Map<Integer, Rotation3d> rotations = Map.of(
        // In order to adjust an AprilTag's rotation, specify non-zero roll/pitch/yaw, as viewed
        // from perspective of the field XYZ axes. For example, if the blue speaker AprilTag is
        // tilted upward by 2 degrees:
        //
        //   7, new Rotation3d(0.0, Units.degreesToRadians(-2.0), 0.0)
        //
        // The same upward tilt for the red speaker AprilTag would be corrected as such:
        //
        //   4, new Rotation3d(0.0, Units.degreesToRadians(2.0), 0.0)
      );
      List<AprilTag> aprilTags =
        rawLayout.getTags().stream()
        .map(aprilTag -> {
          AprilTag rotatedAprilTag;
          if (rotations.containsKey(aprilTag.ID)) {
            rotatedAprilTag =
              new AprilTag(aprilTag.ID, aprilTag.pose.rotateBy(rotations.get(aprilTag.ID)));
          } else {
            rotatedAprilTag = aprilTag;
          }
          return rotatedAprilTag;
        })
        .toList();
      return new AprilTagFieldLayout(
        aprilTags, rawLayout.getFieldLength(), rawLayout.getFieldWidth());
    }
    public static final AprilTagFieldLayout kAprilTagFieldLayout =
      loadTransformedAprilTagFieldLayout();

    public static final double kMaxX = kAprilTagFieldLayout.getFieldLength();

    private static AprilTag getAprilTag(AprilTagFieldLayout fieldLayout, int aprilTagID) {
      for (AprilTag aprilTag : fieldLayout.getTags()) {
        if (aprilTag.ID == aprilTagID) {
          return aprilTag;
        }
      }
      assert(false);
      return new AprilTag(0, null);
    }

    public static final FieldPoseUtil.AprilTags kBlueAprilTags = new FieldPoseUtil.AprilTags(
      getAprilTag(kAprilTagFieldLayout, 21), // reefTwelve
      getAprilTag(kAprilTagFieldLayout, 22), // reefTwo
      getAprilTag(kAprilTagFieldLayout, 17), // reefFour
      getAprilTag(kAprilTagFieldLayout, 18), // reefSix
      getAprilTag(kAprilTagFieldLayout, 19), // reefEight
      getAprilTag(kAprilTagFieldLayout, 20), // reefTen
      getAprilTag(kAprilTagFieldLayout, 13), // coralStationLeft
      getAprilTag(kAprilTagFieldLayout, 12), // coralStationRight
      getAprilTag(kAprilTagFieldLayout, 16), // processor
      getAprilTag(kAprilTagFieldLayout, 14), // bargeLeft
      getAprilTag(kAprilTagFieldLayout, 15)  // bargeRight
    );
    public static final FieldPoseUtil.AprilTags kRedAprilTags = new FieldPoseUtil.AprilTags(
      getAprilTag(kAprilTagFieldLayout, 10), // reefTwelve
      getAprilTag(kAprilTagFieldLayout, 9), // reefTwo
      getAprilTag(kAprilTagFieldLayout, 8), // reefFour
      getAprilTag(kAprilTagFieldLayout, 7), // reefSix
      getAprilTag(kAprilTagFieldLayout, 6), // reefEight
      getAprilTag(kAprilTagFieldLayout, 11), // reefTen
      getAprilTag(kAprilTagFieldLayout, 1), // coralStationLeft
      getAprilTag(kAprilTagFieldLayout, 2), // coralStationRight
      getAprilTag(kAprilTagFieldLayout, 3), // processor
      getAprilTag(kAprilTagFieldLayout, 5), // bargeLeft
      getAprilTag(kAprilTagFieldLayout, 4)  // bargeRight
    );
  }

  public static final class CraneConstants {
    public static final int kLeftElevatorMotorID = 13; // TODO: Change the ID or give that ID to the motor
    public static final int kRightElevatorMotorID = 14; // TODO: Change the ID or give that ID to the motor
    public static final int kPivotMotorID = 15; // TODO: Change the ID or give that ID to the motor
    public static final int kDistanceSensorInput = 4; // TODO: Configure.

    public static final long kValueCacheTtlMicroseconds = 15;

    // Time required to accelerate from stationary to maximum velocity. Acceleration must
    // be proportional between the pivot and elevator in order to achieve "linear" movement,
    // hence the shared constant. Both pivot and elevator must be capable of this acceleration
    // for the crane to move smoothly.
    public static final double kAccelerationSeconds = 0.25;
    public static final double kPivotMaxSpeedRadiansPerSecond = Math.PI;
    public static final double kPivotMaxAccelerationRadiansPerSecondSquared =
      kPivotMaxSpeedRadiansPerSecond / kAccelerationSeconds; // Do not change.
    public static final double kElevatorMaxSpeedMetersPerSecond = 1.0;
    public static final double kElevatorMaxAcccelerationMetersPerSecondSquared =
      kElevatorMaxSpeedMetersPerSecond / kAccelerationSeconds; // Do not change.

    public static final SparkUtil.PIDFSlot kPivotMotorVelocityPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0), // TODO: Tune.
      ClosedLoopSlot.kSlot0
    );
    public static final SparkUtil.PIDFSlot kPivotMotorVoltagePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0), // TODO: Tune.
      ClosedLoopSlot.kSlot1
    );
    public static final SparkUtil.Config kPivotMotorConfig = new SparkUtil.Config(
      20, // TODO: Configure.
      0.1, // TODO: Configure.
      false, // TODO: Find if the pivot motor is reversed
      1.0, // TODO: Compute.
      1.0, // TODO: Compute.
      Math.PI / 2.0,
      2.0 * Math.PI,
      new ArrayList<>() {{
        add(kPivotMotorVelocityPIDFSlot);
        add(kPivotMotorVoltagePIDFSlot);
      }}
    );
    public static final double kPivotHomingVoltage = 1.0; // TODO: Tune.
    public static final double kPivotMinStalledHomingAmperage = 30.0; // TODO: Tune.

    public static final SparkUtil.PIDFSlot kElevatorMotorVelocityPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0), // TODO: Tune.
      ClosedLoopSlot.kSlot0
    );
    public static final SparkUtil.PIDFSlot kElevatorMotorVoltagePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0), // TODO: Tune.
      ClosedLoopSlot.kSlot1
    );
    public static final SparkUtil.Config kElevatorMotorConfig = new SparkUtil.Config(
      20, // TODO: Configure.
      0.1, // TODO: Configure.
      true, // TODO: Find what elevator motor is reversed
      1.0, // TODO: Compute.
      1.0, // TODO: Compute.
      1.0,
      4.0,
      new ArrayList<>() {{
        add(kElevatorMotorVelocityPIDFSlot);
        add(kElevatorMotorVoltagePIDFSlot);
      }}
    );
    public static final double kElevatorHomingVoltage = 1.0; // TODO: Tune.
    public static final double kElevatorMinStalledHomingAmperage = 30.0; // TODO: Tune.

    public static final Crane.Tolerance kDefaultPivotTolerance = new Crane.Tolerance(
      Units.degreesToRadians(0.5),
      Units.degreesToRadians(0.25)
    );
    public static final Crane.Tolerance kDefaultElevatorTolerance = new Crane.Tolerance(
      0.005,
      0.0025
    );

    public static final PIDF kPivotPIDF = new PIDF(0.0, 0.0, 0.0, 0.0); // TODO: Tune.
    public static final PIDF kElevatorPIDF = new PIDF(3.0, 0.0, 0.0, 0.2); // TODO: Tune

    public static final double kPivotHardMax = Units.degreesToRadians(90.5);
    public static final double kPivotHiMin = Units.degreesToRadians(-90.0);
    public static final double kPivotHiMax = Units.degreesToRadians(44.0);
    public static final double kPivotLoMin = Units.degreesToRadians(-15.0);
    public static final double kPivotLoMax = Units.degreesToRadians(90.0);
    public static final double kPivotHome = kPivotLoMax;

    public static final double kElevatorMax = 1.400;
    public static final double kElevatorHiPivotHome = 1.250;
    public static final double kElevatorLoHiThreshold = 0.900;
    public static final double kElevatorHomeRapid = 0.250;
    public static final double kElevatorMin = 0.220;
    public static final double kElevatorHome = kElevatorMin;
    public static final double kElevatorHardMin = 0.201;

    // Valid configuration space boundaries.
    public static final Segment kPivotLoBoundary = new Segment(
      new Translation2d(kPivotHiMin, kElevatorMax),
      new Translation2d(kPivotLoMin, kElevatorMin)
    );
    public static final Segment kPivotHiBoundary = new Segment(
      new Translation2d(kPivotHiMax, kElevatorMax),
      new Translation2d(kPivotLoMax, kElevatorMin)
    );
    public static final Segment kElevatorLoBoundary = new Segment(
      new Translation2d(kPivotLoMin, kElevatorMin),
      new Translation2d(kPivotLoMax, kElevatorMin)
    );
    public static final Segment kElevatorHiBoundary = new Segment(
      new Translation2d(kPivotHiMin, kElevatorMax),
      new Translation2d(kPivotHiMax, kElevatorMax)
    );
    public static final ArrayList<Segment> kBoundaries = new ArrayList<>() {{
      add(kPivotLoBoundary);
      add(kPivotHiBoundary);
      add(kElevatorLoBoundary);
      add(kElevatorHiBoundary);
    }};

    // Actual elevator height for a distance sensor measurement of 0.
    public static final double kDistanceSensorBaseMeasurement = 0.150; // TODO: Calibrate.

    public static final Translation2d kPositionHome =
      new Translation2d(kPivotHome, kElevatorHome);
    public static final Translation2d kPositionL1 =
      new Translation2d(Units.degreesToRadians(15.0), 0.454);
    public static final Translation2d kPositionL2 =
      new Translation2d(Units.degreesToRadians(-35.0), 0.975);
    public static final Translation2d kPositionL3 =
      new Translation2d(Units.degreesToRadians(-35.0), 1.378);
    public static final Translation2d kPositionLoAlgae =
      new Translation2d(Units.degreesToRadians(-40.0), 0.924);
    public static final Translation2d kPositionHiAlgae =
      new Translation2d(Units.degreesToRadians(-40.0), 1.328);
    public static final double kIntakeAngle = 55;
    public static final double kIntakeHeight = 0.95; //TODO: calculate elevator height taking into account angle and stinger length
  }
}
