// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.AbsoluteEncoderConfig;
//import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.sim.SparkRelativeEncoderSim;
//import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
//import frc.robot.Constants.ModuleConstants;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private SparkMaxConfig m_drivingConfig;
  private SparkMaxConfig m_turningConfig;
  private DigitalInput m_dutyCycleInput;
  private DutyCycle m_dutyCycleEncoder;
  private double m_chassisAngularOffset = 0;
  private final double m_steerOffset;
  private static final int kEncoderResolution = 4096;
  private static final double kWheelRadius = 2.0 * 0.0254;
  private static final double kGearboxRatio = 1.0 / 6.12;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private static final double kWheelCircumference = 0.3;
  private static final double kDrivePositionFactor = kWheelCircumference * kGearboxRatio;
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset,int dutyCycle, double steerOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    m_steerOffset = steerOffset;
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    //SparkMaxConfig config = new SparkMaxConfig();
    //config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    //Type.kDutyCycle
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    m_dutyCycleInput=new DigitalInput(dutyCycle);
    m_dutyCycleEncoder= new DutyCycle(m_dutyCycleInput);
    m_drivingConfig = new SparkMaxConfig();
    m_turningConfig = new SparkMaxConfig();
    m_drivingConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast);
    m_turningConfig
        .apply(m_drivingConfig);
    //INTergrated version of encoder
    m_drivingConfig.encoder.positionConversionFactor(kDrivePositionFactor).velocityConversionFactor(kDrivePositionFactor/60);
    m_turningConfig.encoder.positionConversionFactor(2*Math.PI);


    m_drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(.1)
    .i(0)
    .d(0)
    .outputRange(-1, 1)
    .p(0.0001,ClosedLoopSlot.kSlot1)
    .i(0,ClosedLoopSlot.kSlot1)
    .d(0,ClosedLoopSlot.kSlot1)
    .velocityFF(1/5767,ClosedLoopSlot.kSlot1)
    .outputRange(-1,1,ClosedLoopSlot.kSlot1);


    m_turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(.1)
    .i(0)
    .d(0)
    .outputRange(-1, 1)
    .p(0.0001,ClosedLoopSlot.kSlot1)
    .i(0,ClosedLoopSlot.kSlot1)
    .d(0,ClosedLoopSlot.kSlot1)
    .velocityFF(1/5767,ClosedLoopSlot.kSlot1)
    .outputRange(-1,1,ClosedLoopSlot.kSlot1);

    m_drivingSparkMax.configure(m_drivingConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

    
    
    //DEPRECATED
    //m_drivingPIDController.
    //m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    //m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    
    //m_drivingEncoder.setPositionConversionFactor(kDrivePositionFactor);
    //m_drivingEncoder.setVelocityConversionFactor(kDrivePositionFactor/60);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    //ill uncomment and fix if needed
    //m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    //m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    
    //m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.

    //m_turningPIDController.setPositionPIDWrappingEnabled(true);
    //m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!


    //m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    //m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    //m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    //m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    //m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
    //    ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    //m_turningPIDController.setP(ModuleConstants.kTurningP);
    //m_turningPIDController.setI(ModuleConstants.kTurningI);
    //m_turningPIDController.setD(ModuleConstants.kTurningD);
    //m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    //m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    //    ModuleConstants.kTurningMaxOutput);

    //m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    //m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    //m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    //m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    //m_drivingSparkMax.burnFlash();
    //m_turningSparkMax.burnFlash();

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
  public double getAbsoluteEncoderPosition(){
    double initalPosition = m_dutyCycleEncoder.getOutput();
    double initalPositionInRadians = initalPosition * 2.0 * Math.PI;
    double initalPositionInRadiansScaled = new Rotation2d(initalPositionInRadians - m_steerOffset).getRadians();
    return initalPositionInRadiansScaled;
  }
  public void resetAngleEncoderToAbsolute()
    {
        m_turningEncoder.setPosition(getAbsoluteEncoderPosition());
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
    //DEPRECATED
    //SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
       // new Rotation2d(m_turningEncoder.getPosition()));
    Rotation2d currentRotation =new Rotation2d(m_turningEncoder.getPosition());
    correctedDesiredState.optimize(currentRotation);
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
