/*// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.DoublePublisher;
//import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.AbsoluteEncoderConfig;
//import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.sim.SparkRelativeEncoderSim;
//import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import frc.robot.Constants.ModuleConstants;


public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private final double m_chassisAngularOffset;
  // private static final double kWheelRadius = 2.0 * 0.0254;
  private static final double kGearboxRatio = 1.0 / 6.12;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private static final double kWheelCircumference = 0.3;
  private static final double kDrivePositionFactor = kWheelCircumference * kGearboxRatio;
  private static final SparkMaxConfig m_drivingConfig = new SparkMaxConfig();
  private static final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

  private final double turningFactor = 2 * Math.PI;

  private final DoublePublisher m_absoluteEncoderPub;
  private final DoublePublisher m_targetAnglePub;
  private final DoublePublisher m_targetYSpeedPub;
  

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   *
  public MAXSwerveModule(int motorAssembly, double angOffset)
    {
        this(motorAssembly, motorAssembly+ 10, angOffset);
    }
  private MAXSwerveModule(int drivingCANId, int turningCANId, double angOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
    m_chassisAngularOffset=angOffset;
    //double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
    m_drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
    m_drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

    m_turningConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
            // .inverted(true)
            // .voltageCompensation(12.6);
     m_turningConfig.absoluteEncoder
             .inverted(true)
             .positionConversionFactor(turningFactor) // radians
             .velocityConversionFactor(turningFactor / 60.0); // radians per second
    m_turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(1, 0, 0,0)
            .outputRange(-1, 1);
            // .iZone(1);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.

    m_turningConfig.closedLoop.positionWrappingEnabled(true);
    m_turningConfig.closedLoop.positionWrappingMinInput(0);
    m_turningConfig.closedLoop.positionWrappingMaxInput(turningFactor);
   
    m_drivingSparkMax.configure(m_drivingConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
      
    m_desiredState.angle = getAbsoluteAngle();
    m_drivingEncoder.setPosition(0);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    var dataTable = inst.getTable("swerveMod");
    var name = "Mod " + turningCANId;
    m_absoluteEncoderPub = dataTable.getDoubleTopic(name + "/absolute angle").publish();
    m_targetAnglePub = dataTable.getDoubleTopic(name + "/target angle").publish();
    m_targetYSpeedPub = dataTable.getDoubleTopic(name + "/target Y Speed").publish();
  }

  public void dashboardUpdate()
  {
    m_absoluteEncoderPub.set(getAbsoluteAngle().getDegrees());
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   *
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), getAbsoluteAngleWithChassesOffset() );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   *
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        getAbsoluteAngleWithChassesOffset());
  }

  public double getDriveVelocity()
  {
        return m_drivingEncoder.getVelocity() * kDrivePositionFactor;
  }

    private void setDriveVelocity(double targetSpeed)
    {
      SmartDashboard.putNumber("Swerve target speed", targetSpeed);
        m_drivingPIDController.setReference(targetSpeed, ControlType.kVelocity);
    }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   *
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));   
    correctedDesiredState.optimize(getAbsoluteAngle());

    setDriveVelocity(desiredState.speedMetersPerSecond);
    setSteerAngleInRadians(correctedDesiredState.angle.getRadians());
    m_targetYSpeedPub.set(desiredState.speedMetersPerSecond);
    m_desiredState = desiredState;
  }

  private void setSteerAngleInRadians(double targetAngleInRadians)
  {
    m_targetAnglePub.set(Rotation2d.fromRadians(targetAngleInRadians).getDegrees());
    m_turningPIDController.setReference(targetAngleInRadians, SparkMax.ControlType.kPosition);
  }

    /** Zeroes all the SwerveModule encoders. *
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

   private Rotation2d getAbsoluteAngle()
   {
     return new Rotation2d(m_turningEncoder.getPosition());
   }

   private Rotation2d getAbsoluteAngleWithChassesOffset()
   {
     return new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset);
   }
}
*/

//START OF TEMPLATE CODE
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}