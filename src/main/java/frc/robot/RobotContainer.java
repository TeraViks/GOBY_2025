// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HandlerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralPlacement;
import frc.robot.commands.FaceReef;
import frc.robot.commands.FaceStation;
import frc.robot.commands.GetCoral;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final CameraSubsystem m_cameraSystem =
    CameraConstants.kEnable ? new CameraSubsystem(CameraConstants.kCameraConfigs) : null;
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_cameraSystem);
  private final HandlerSubsystem m_handler = new HandlerSubsystem(
    HandlerConstants.kMotorID, HandlerConstants.kmotorConfig);
  private final Crane m_crane = new Crane();
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);
  FieldPoseUtil m_fieldPoseUtil = new FieldPoseUtil();
  double m_reverseFactor = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;

  private CoralStationSubPose m_selectedCoralStationSlot = CoralStationSubPose.FIVE;

  private static double joystickTransform(double value) {
    double transformedValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    if (DriveConstants.kSquareInputs) {
      transformedValue = transformedValue * Math.abs(transformedValue);
    }
    return transformedValue;
  }

  private double getXSpeedInput() {
    // The Y axis on the controller is inverted! Pushing forward (up) generates a negative value.
    // Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))
      * OIConstants.kMaxMetersPerSec;
  }

  private double getYSpeedInput() {
    // The X axis on the controller behaves as expected (right is positive), but we're using it for
    // Y axis control, where left is positive. Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))
      * OIConstants.kMaxMetersPerSec;
  }

  private double getRotationSpeedInput() {
    // Moving the joystick to the right causes positive input, which we negate in order to rotate
    // clockwise.
    return -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))
      * OIConstants.kMaxRadPerSec;
  }

  public RobotContainer() {
    configureBindings();

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> {
          m_robotDrive.drive(
            getXSpeedInput(),
            getYSpeedInput(),
            getRotationSpeedInput(),
            true);
        }, m_robotDrive
      )
    );

    NamedCommands.registerCommand("RightHourTwoAuto",
      new CoralPlacement(m_robotDrive, m_handler, m_crane, ReefSubPose.A, CraneConstants.kPositionL2));
    NamedCommands.registerCommand("LeftHourTenAuto",
      new CoralPlacement(m_robotDrive, m_handler, m_crane, ReefSubPose.A, CraneConstants.kPositionL2));
    m_chooser.setDefaultOption("Empty Auto", new PathPlannerAuto("Empty Auto"));
    m_chooser.addOption("Middle Cross The Line", new PathPlannerAuto("Middle Cross The Line"));
    m_chooser.addOption("Right auto with place", new PathPlannerAuto("Right Start and Place"));
    m_chooser.addOption("Left auto with place", new PathPlannerAuto("Left Start and Place"));
    SmartDashboard.putData(m_chooser);
  }

  public void initializePreloaded() {
    m_handler.initializePreloaded();
  }

  public void setPIDSlotID(ClosedLoopSlot slotID) {
    m_robotDrive.setPIDSlotID(slotID);
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, OIConstants.kZeroGyro)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> {
        m_robotDrive.zeroGyro();
      }, m_robotDrive
    ));

    new JoystickButton(m_driverController, OIConstants.kFaceReefButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new FaceReef(
        m_robotDrive,
        () -> getXSpeedInput(),
        () -> getYSpeedInput(),
        m_fieldPoseUtil));

    new JoystickButton(m_driverController, OIConstants.kFaceCoralStationButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new FaceStation(
        m_robotDrive,
        () -> getXSpeedInput(),
        () -> getYSpeedInput(),
        m_fieldPoseUtil));

    new Trigger(() -> m_driverController.getPOV() == OIConstants.kCoralIntake2)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> m_selectedCoralStationSlot = CoralStationSubPose.TWO));

    new Trigger(() -> m_driverController.getPOV() == OIConstants.kCoralIntake5)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> m_selectedCoralStationSlot = CoralStationSubPose.FIVE));

    new Trigger(() -> m_driverController.getPOV() == OIConstants.kCoralIntake8)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> m_selectedCoralStationSlot = CoralStationSubPose.EIGHT));

    new JoystickButton(m_driverController, OIConstants.kFaceProcessorButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new Command() {}); //TODO: Create a command to align to the processor

      new Trigger(() ->
          m_operatorController.getRawButton(OIConstants.kASideButton) &&
          m_operatorController.getRawButton(OIConstants.kLevel2Button))
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(new CoralPlacement(
          m_robotDrive, m_handler, m_crane, ReefSubPose.A, CraneConstants.kPositionL2));

      new Trigger(() ->
          m_operatorController.getRawButton(OIConstants.kASideButton) &&
          m_operatorController.getRawButton(OIConstants.kLevel3Button))
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(new CoralPlacement(
          m_robotDrive, m_handler, m_crane, ReefSubPose.A, CraneConstants.kPositionL3));

      new Trigger(() ->
          m_operatorController.getRawButton(OIConstants.kBSideButton) &&
          m_operatorController.getRawButton(OIConstants.kLevel2Button))
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(new CoralPlacement(
          m_robotDrive, m_handler, m_crane, ReefSubPose.B, CraneConstants.kPositionL2));

      new Trigger(() ->
          m_operatorController.getRawButton(OIConstants.kBSideButton) &&
          m_operatorController.getRawButton(OIConstants.kLevel3Button))
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(new CoralPlacement(
          m_robotDrive, m_handler, m_crane, ReefSubPose.B, CraneConstants.kPositionL3));

      new JoystickButton(m_operatorController, OIConstants.kLevel1Button)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_crane.moveTo(FieldConstants.kLevel1)
        }, m_robotDrive)); //TODO: Requirements also need to include the crane subsystem

      new Trigger(() -> m_operatorController.getPOV() == OIConstants.kHighAlgaePOV)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_crane.moveTo(FieldConstants.kHighAlgae)
        })); //TODO: Requirements also need to include the crane subsystem

      new Trigger(() -> m_operatorController.getPOV() == OIConstants.kLowAlgaePOV)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_crane.moveTo(FieldConstants.kLowAlgae)
        })); //TODO: Requirements also need to include the crane subsystem

      new Trigger(() -> m_operatorController.getPOV() == OIConstants.kAlgaeProcessorPOV)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_crane.moveTo(FieldConstants.kAlgaeProcessor)
          //Robot also needs to move to right position
        })); //TODO: Requirements also need to include the crane subsystem

      new Trigger(() -> m_operatorController.getPOV() == OIConstants.kAlgaeProcessorPOV)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_crane.moveTo(FieldConstants.kAlgaeProcessor)
          //Robot also needs to move to right position
        })); //TODO: Requirements also need to include the crane subsystem

      new Trigger(() -> m_operatorController.getPOV() == OIConstants.kIntakeALgaePOV)
        .debounce(OIConstants.kDebounceSeconds)
        .whileTrue(Commands.runOnce(() -> {
          m_handler.intakeAlgae();
        }, m_handler))
        .onFalse(Commands.runOnce(() -> {
          m_handler.cancelIntake();
        }, m_handler));

      new JoystickButton(m_operatorController, OIConstants.kIntakeCoralButton)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(new GetCoral(m_robotDrive, m_handler, m_crane, m_selectedCoralStationSlot));

      new JoystickButton(m_operatorController, OIConstants.kEjectButton)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          m_handler.eject();
        }, m_handler));

      new Trigger(() ->
          m_operatorController.getRawAxis(OIConstants.kExtendClimberAxis) >=
          OIConstants.kTriggerAcuationValue)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_climber.extendClimber()
        })); //TODO: Requirements needs to include m_climber

      new Trigger(() ->
          m_operatorController.getRawAxis(OIConstants.kRetractClimberAxis) >=
          OIConstants.kTriggerAcuationValue)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(Commands.runOnce(() -> {
          //m_climber.retractClimber()
        })); //TODO: Requirements needs to include m_climber
    }

  public Command getAutonomousCommand() {
    Command selectedCommand = m_chooser.getSelected();
    return selectedCommand;
  }
}