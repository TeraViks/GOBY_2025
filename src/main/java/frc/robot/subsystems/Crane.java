package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CraneConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.Segment;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.SparkUtil.PIDFSlot;
import frc.robot.utilities.TunableDouble;
import frc.robot.utilities.TunablePIDF;
import frc.robot.utilities.ValueCache;
import frc.robot.utilities.Vector;

/** The Crane subsystem controls simultaneous movement along pivot and elevator axes.
 *  Mathematically this is modeled as a plane, where the pivot angle in radians is the x axis,
 *  and the elevator height in meters is the y axis. Thus a Transform2d is an (angle,height) point,
 *  aka (a,h). "Positions" are at "points".
 */
public class Crane extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlex m_rightElevatorMotor;

  private final RelativeEncoder m_pivotEncoder;
  private final DutyCycleEncoder m_pivotDutyCycleEncoder;
  private final RelativeEncoder m_elevatorEncoder;

  private final SparkClosedLoopController m_pivotPID;
  private final SparkClosedLoopController m_leftElevatorPID;

  private final Pololu4079 m_distanceSensor;

  private final ValueCache<Double> m_pivotPositionCache;
  private final ValueCache<Double> m_pivotVelocityCache;
  private final ValueCache<Double> m_elevatorPositionCache;
  private final ValueCache<Double> m_elevatorVelocityCache;

  private static final TunablePIDF pivotPIDF = new TunablePIDF("Crane.pivotPIDF",
    CraneConstants.kPivotPIDF);
  private static final TunablePIDF elevatorPIDF = new TunablePIDF("Crane.elevatorPIDF",
    CraneConstants.kElevatorPIDF);
  private static final TunablePIDF pivotVoltagePIDF = new TunablePIDF("Crane.pivotVoltagePIDF",
    CraneConstants.kPivotMotorVoltagePIDFSlot.pidf());
  private static final TunablePIDF elevatorVoltagePIDF = new TunablePIDF("Crane.elevatorVoltagePIDF",
    CraneConstants.kElevatorMotorVoltagePIDFSlot.pidf());
  private static final TunablePIDF pivotVelocityPIDF = new TunablePIDF("Crane.pivotVelocityPIDF",
    CraneConstants.kPivotMotorVelocityPIDFSlot.pidf());
  private static final TunablePIDF elevatorVelocityPIDF = new TunablePIDF("Crane.elevatorVelocityPIDF",
    CraneConstants.kElevatorMotorVelocityPIDFSlot.pidf());

  private ProfiledPIDController m_aController = new ProfiledPIDController(
    pivotPIDF.get().p(),
    pivotPIDF.get().i(),
    pivotPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  private ProfiledPIDController m_hController = new ProfiledPIDController(
    elevatorPIDF.get().p(),
    elevatorPIDF.get().i(),
    elevatorPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  
  private static final TunableDouble kS = new TunableDouble("Crane.kS", CraneConstants.kS);
  private static final TunableDouble kG = new TunableDouble("Crane.kG", CraneConstants.kG);
  private static final TunableDouble kV = new TunableDouble("Crane.kV", CraneConstants.kV);
  private ElevatorFeedforward m_elevatorFF = new ElevatorFeedforward(kS.get(), kG.get(), kV.get());

  private Translation2d m_setpoint = new Translation2d(0.0, 0.0);
  private double m_pivotControlFactor; // 1.0 for position-based control.
  private double m_elevatorControlFactor; // 1.0 for position-based control.

  /** Tolerance. Position is in {radians,meters}. Velocity is in {radians,meters}/second. */
  public record Tolerance(double position, double velocity) {}

  private int m_currentSerialNum = 0;
  private boolean m_isVelocityControlled = false;

  private boolean m_isElevatorHomed = false;
  private boolean m_isPivotHomed = false;

  private enum State {
    CRANING,             // Normal crane operation.
    ESTIMATE_H,          // Estimate elevator height to inform homing
                         // Low homing mode:
    LO_PIVOT_HOME,       //   Home the pivot.
    LO_ELEVATOR_RAPID,   //   Move close to elevator home
                         // High homing mode
    HI_PIVOT_SAFE,       //   Move the pivot to a safe position to allow elevator homing.
    HI_ELEVATOR_RAPID,   //   Move close to elevator home.
    ELEVATOR_HOME,       // Home the elevator.
  }
  private State m_state;

  public Crane() {
    m_pivotMotor = new SparkFlex(CraneConstants.kPivotMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_pivotMotor, CraneConstants.kPivotMotorConfig);

    m_leftElevatorMotor = new SparkFlex(CraneConstants.kLeftElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_leftElevatorMotor, CraneConstants.kElevatorMotorConfig);

    m_rightElevatorMotor = new SparkFlex(CraneConstants.kRightElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureFollowerMotor(
      m_rightElevatorMotor,
      CraneConstants.kElevatorMotorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
      m_leftElevatorMotor);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(0.0);
    m_pivotDutyCycleEncoder = new DutyCycleEncoder(5, (2 * Math.PI), 2.0); //TODO: tune expected zero, set channel
    m_elevatorEncoder = m_leftElevatorMotor.getEncoder();
    m_elevatorEncoder.setPosition(0.0);

    m_pivotPID = m_pivotMotor.getClosedLoopController();
    m_leftElevatorPID = m_leftElevatorMotor.getClosedLoopController();

    m_distanceSensor = new Pololu4079(CraneConstants.kDistanceSensorInput);

    m_pivotPositionCache =
      new ValueCache<Double>(() -> {
        return m_pivotEncoder.getPosition();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_pivotVelocityCache =
      new ValueCache<Double>(() -> {
        return m_pivotEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorPositionCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getPosition();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorVelocityCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);

    if (Constants.kCompeting) {
      toStateLoPivotHome();
    } else {
      m_state = State.ESTIMATE_H;
    }
  }

  // Only increase serial number once while controlling with velocity to avoid the serial
  // number rapidly increasing.
  private int allocateSerialNum(boolean velocityControl) {
    if (velocityControl) {
      if (!m_isVelocityControlled) {
        m_isVelocityControlled = true;
        m_currentSerialNum++;
      }
    } else {
      m_isVelocityControlled = false;
      m_currentSerialNum++;
    }
    return m_currentSerialNum;
  }

  private int moveTo(Translation2d setpoint,
      Tolerance pivotTolerance, Tolerance elevatorTolerance,
      double pivotVelocityFactor, double elevatorVelocityFactor) {
    m_setpoint = setpoint;
    m_aController.setGoal(m_setpoint.getX());
    m_hController.setGoal(m_setpoint.getY());
    m_aController.setTolerance(pivotTolerance.position,
      Constants.kDt * pivotTolerance.velocity);
    m_hController.setTolerance(elevatorTolerance.position,
      Constants.kDt * elevatorTolerance.velocity
    );
    m_pivotControlFactor = pivotVelocityFactor;
    m_elevatorControlFactor = elevatorVelocityFactor;
    boolean velocityControl = pivotVelocityFactor != 1.0 || elevatorVelocityFactor != 1.0;
    return allocateSerialNum(velocityControl);
  }

  public int moveTo(Translation2d setpoint,
      Tolerance pivTolerance, Tolerance elevatorTolerance) {
    return moveTo(setpoint, pivTolerance, elevatorTolerance,
      1.0, 1.0);
  }

  public int moveTo(Translation2d setpoint) {
    return moveTo(setpoint,
      CraneConstants.kDefaultPivotTolerance, CraneConstants.kDefaultElevatorTolerance);
  }

  private void updatePivotSetpoint(double pivotAngle) {
    Translation2d setpoint = new Translation2d(pivotAngle, m_setpoint.getY());
    m_setpoint = setpoint;
  }

  public int movePivotTo(double pivotAngle) {
    Translation2d setpoint = new Translation2d(pivotAngle, m_setpoint.getY());
    return moveTo(setpoint);
  }

  private void updateElevatorSetpoint(double elevatorHeight) {
    Translation2d setpoint = new Translation2d(m_setpoint.getX(), elevatorHeight);
    m_setpoint = setpoint;
  }

  public int moveElevatorTo(double elevatorHeight) {
    Translation2d setpoint = new Translation2d(m_setpoint.getX(), elevatorHeight);
    return moveTo(setpoint);
  }

  private void moveTo(Translation2d setpoint,
      double pivotVelocityFactor, double elevatorVelocityFactor) {
    moveTo(setpoint,
      CraneConstants.kDefaultPivotTolerance, CraneConstants.kDefaultElevatorTolerance,
      pivotVelocityFactor, elevatorVelocityFactor);
  }

  public void move(double pivotVelocityFactor, double elevatorVelocityFactor) {
    if (pivotVelocityFactor != 0.0 || elevatorVelocityFactor != 0.0) {
      Translation2d velocity = new Translation2d(
        pivotVelocityFactor * CraneConstants.kPivotMaxSpeedRadiansPerSecond,
        elevatorVelocityFactor * CraneConstants.kElevatorMaxSpeedMetersPerSecond
      );
      Vector v = new Vector(getPosition(), velocity.getAngle());
      for (Segment boundary : CraneConstants.kBoundaries) {
        Optional<Translation2d> setpointOpt = boundary.intersection(v);
        if (setpointOpt.isPresent()) {
          Translation2d setpoint = setpointOpt.get();
          moveTo(setpoint, pivotVelocityFactor, elevatorVelocityFactor);
          return;
        }
      }
      System.out.printf("move(%.2f, %.2f) missed all boundaries\n",
        pivotVelocityFactor, elevatorVelocityFactor);
    }

    // Zero velocity, or outside boundaries; move to (i.e. stay at) current position.
    moveTo(getPosition());
  }

  public void movePivot(double pivotVelocityRadiansPerSecond) {
    move(pivotVelocityRadiansPerSecond, 0.0);
  }

  public void moveElevator(double elevatorVelocityMetersPerSecond) {
    move(0.0, elevatorVelocityMetersPerSecond);
  }

  private boolean atSetpointImpl() {
    return m_aController.atSetpoint() && m_hController.atSetpoint();
  }

  public Optional<Integer> atSetpoint() {
    if (atSetpointImpl()) {
      return Optional.of(m_currentSerialNum);
    } else {
      return Optional.empty();
    }
  }

  /* Dynamically scale the a,h controller constraints such that the combined a,h component
   * movements combine to follow a "straight" line, i.e. the component movements complete
   * simultaneously. */
  private void scaleAHConstraints(Translation2d position, Translation2d deviation) {
    // Estimate pivot,elevator movement time, ignoring current velocity, as the basis of constraint
    // factors. Acceleration can be legitimately ignored since it proportionally affects the axes.
    double pivotTime = Math.abs(deviation.getX())
      / (CraneConstants.kPivotMaxSpeedRadiansPerSecond * m_pivotControlFactor);
    double elevatorTime = Math.abs(deviation.getY())
      / CraneConstants.kElevatorMaxSpeedMetersPerSecond * m_elevatorControlFactor;
    double maxTime = Math.max(pivotTime, elevatorTime);
    double aFactor = pivotTime / maxTime;
    double hFactor = elevatorTime / maxTime;
    m_aController.setConstraints(new TrapezoidProfile.Constraints(
      aFactor * CraneConstants.kPivotMaxSpeedRadiansPerSecond * m_pivotControlFactor,
      aFactor * CraneConstants.kPivotMaxAccelerationRadiansPerSecondSquared * m_pivotControlFactor
    ));
    m_hController.setConstraints(new TrapezoidProfile.Constraints(
      hFactor * CraneConstants.kElevatorMaxSpeedMetersPerSecond * m_elevatorControlFactor,
      hFactor * CraneConstants.kElevatorMaxAcccelerationMetersPerSecondSquared * m_elevatorControlFactor
    ));
  }

  private void resetCrane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);
    Translation2d velocity = getVelocity();

    scaleAHConstraints(position, deviation);
    m_aController.reset(
      position.getX(),
      velocity.getX()
    );
    m_hController.reset(
      position.getY(),
      velocity.getY()
    );
  }

  private void movePivotWithAbsEncoder(double pivotAngle) {
    Translation2d position = getPositionWithAbsEncoder();
    Translation2d deviation = getDeviation(position);
    
    m_aController.setGoal(pivotAngle);

    scaleAHConstraints(position, deviation);
    double aVelocity = m_aController.calculate(position.getX());
    SmartDashboard.putNumber("Pivot Goal", Units.radiansToDegrees(m_aController.getGoal().position));
    m_pivotPID.setReference(aVelocity, ControlType.kMAXMotionVelocityControl,
      CraneConstants.kPivotMotorVelocityPIDFSlot.slot());
  }
  
  private void crane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);

    scaleAHConstraints(position, deviation);
    double aVelocity = m_aController.calculate(position.getX());
    double hVelocity = m_hController.calculate(position.getY());
    m_pivotPID.setReference(aVelocity, ControlType.kMAXMotionVelocityControl,
      CraneConstants.kPivotMotorVelocityPIDFSlot.slot());
    m_leftElevatorPID.setReference(hVelocity, ControlType.kMAXMotionVelocityControl,
      CraneConstants.kElevatorMotorVelocityPIDFSlot.slot(),
      m_elevatorFF.calculateWithVelocities(m_elevatorEncoder.getVelocity(), m_hController.getSetpoint().velocity));
  }

  private void initPivotPosition(double a) {
    m_pivotEncoder.setPosition(a);
    m_pivotPositionCache.flush();
    updatePivotSetpoint(a);
    resetCrane();
  }

  private void initElevatorPosition(double h) {
    m_elevatorEncoder.setPosition(h);
    m_elevatorPositionCache.flush();
    updateElevatorSetpoint(h);
    resetCrane();
  }

  private Translation2d getPosition() {
    return new Translation2d(m_pivotPositionCache.get(), m_elevatorPositionCache.get());
  }

  private Translation2d getPositionWithAbsEncoder() {
    return new Translation2d(getAbsoluteEncoderRadians(), m_elevatorPositionCache.get());
  }

  private Translation2d getVelocity() {
    return new Translation2d(m_pivotVelocityCache.get(), m_elevatorVelocityCache.get());
  }

  private Translation2d getDesiredTranslation() {
    return m_setpoint;
  }

  private Translation2d getDeviation(Translation2d position) {
    return getDesiredTranslation().minus(position);
  }

  private double getAbsoluteEncoderRadians() {
    return MathUtil.angleModulus(Units.rotationsToRadians(m_pivotDutyCycleEncoder.get()));
  }

  private void toStateCraning() {
    m_state = State.CRANING;
  }

  private void toStateLoPivotHome() {
    movePivotWithAbsEncoder(Math.PI / 2.0);
    m_state = State.LO_PIVOT_HOME;
  }

  private void toStateLoElevatorRapid() {
    double h = m_elevatorPositionCache.get();
    if (h > CraneConstants.kElevatorHomeRapid) {
      moveElevatorTo(CraneConstants.kElevatorHomeRapid);
      m_state = State.LO_ELEVATOR_RAPID;
    } else {
      toStateElevatorHome();
    }
  }

  private void toStateHiPivotSafe() {
    movePivotWithAbsEncoder(CraneConstants.kPivotHiMax);
    m_state = State.HI_PIVOT_SAFE;
  }

  private void toStateHiElevatorRapid() {
    moveElevatorTo(CraneConstants.kElevatorHomeRapid);
    m_state = State.HI_ELEVATOR_RAPID;
  }

  private void toStateElevatorHome() {
    // Use low voltage to move downward slowly.
    m_leftElevatorPID.setReference(CraneConstants.kElevatorHomingVoltage, ControlType.kVoltage,
      CraneConstants.kElevatorMotorVoltagePIDFSlot.slot());
    m_state = State.ELEVATOR_HOME;
  }

  private void updateConstants() {
    if (pivotPIDF.hasChanged()) {
      PIDF pidf = pivotPIDF.get();
      m_aController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    if (elevatorPIDF.hasChanged()) {
      PIDF pidf = elevatorPIDF.get();
      m_hController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    if (elevatorVoltagePIDF.hasChanged()) {
      PIDF pidf = elevatorVoltagePIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kElevatorMotorVoltagePIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(elevatorVelocityPIDF.get(), CraneConstants.kElevatorMotorVelocityPIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kElevatorMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_leftElevatorMotor, motorConfig);
      SparkUtil.configureFollowerMotor(
        m_rightElevatorMotor,
        motorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
        m_leftElevatorMotor
      );
    }
    if (elevatorVelocityPIDF.hasChanged()) {
      PIDF pidf = elevatorVelocityPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kElevatorMotorVelocityPIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(elevatorVoltagePIDF.get(), CraneConstants.kElevatorMotorVoltagePIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kElevatorMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_leftElevatorMotor, motorConfig);
      SparkUtil.configureFollowerMotor(
        m_rightElevatorMotor,
        motorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
        m_leftElevatorMotor
      );
    }
    if (pivotVoltagePIDF.hasChanged()) {
      PIDF pidf = pivotVoltagePIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kPivotMotorVoltagePIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(pivotVelocityPIDF.get(), CraneConstants.kPivotMotorVelocityPIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kPivotMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_pivotMotor, motorConfig);
    }
    if (pivotVelocityPIDF.hasChanged()) {
      PIDF pidf = pivotVelocityPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kPivotMotorVelocityPIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(pivotVoltagePIDF.get(), CraneConstants.kPivotMotorVoltagePIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kPivotMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_pivotMotor, motorConfig);
    }
    
    if (kS.hasChanged()) {
      m_elevatorFF.setKs(kS.get());
    }
    if (kG.hasChanged()) {
      m_elevatorFF.setKg(kG.get());
    }
    if (kV.hasChanged()) {
      m_elevatorFF.setKv(kV.get());
    }
  }

  @Override
  public void periodic() {
    updateConstants();
    SmartDashboard.putString("Crane state", m_state.toString());
    switch (m_state) {
      case CRANING: {
        crane();
        break;
      }
      case ESTIMATE_H: {
        double h = CraneConstants.kDistanceSensorBaseMeasurement + m_distanceSensor.getDistance();
        initElevatorPosition(h);
        if (h <= CraneConstants.kElevatorLoHiThreshold) {
          toStateLoPivotHome();
        } else {
          toStateHiPivotSafe();
        }
        break;
      }
      case LO_PIVOT_HOME:
          crane();
          if (atSetpointImpl()) {
            initPivotPosition(CraneConstants.kPivotHome);
            m_isPivotHomed = true;
            if (m_isElevatorHomed) {
              toStateCraning();
            } else {
              toStateLoElevatorRapid();
            }  
          }
          break;
      case LO_ELEVATOR_RAPID: {
        crane();
        if (atSetpointImpl()) {
          toStateLoElevatorRapid();
        }
        break;
      }
      case HI_PIVOT_SAFE: {
        crane();
        if (atSetpointImpl()) {
          toStateHiElevatorRapid();
        }
        break;
      }
      case HI_ELEVATOR_RAPID: {
        crane();
        if (atSetpointImpl()) {
          toStateElevatorHome();
        }
        break;
      }
      case ELEVATOR_HOME: {
        // Check if motor amperage is spiked due to a stall condition creating a short circuit.
        if (m_leftElevatorMotor.getOutputCurrent() >= CraneConstants.kElevatorMinStalledHomingAmperage) {
          m_leftElevatorMotor.stopMotor();
          initElevatorPosition(CraneConstants.kElevatorHardMin);
          moveElevatorTo(CraneConstants.kElevatorHome);
          m_isElevatorHomed = true;
          if (m_isPivotHomed) {
            toStateCraning();
          } else {
            toStateLoPivotHome();
          }
        }
        break;
      }
    }
  }
}