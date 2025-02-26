// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ModuleConstants;

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
//import frc.robot.Constants.ModuleConstants;


public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  // private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private final double m_angOffset;
  // private static final double kWheelRadius = 2.0 * 0.0254;
  private static final double kGearboxRatio = 1.0 / 6.12;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private static final double kWheelCircumference = 0.3;
  private static final double kDrivePositionFactor = kWheelCircumference * kGearboxRatio;
  private static final SparkMaxConfig m_drivingConfig = new SparkMaxConfig();
  private static final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

  private final double turningFactor = 2 * Math.PI;

  //private final DoublePublisher m_absoluteEncoderPub;
  private final DoublePublisher m_relativeEncoderPub;
  private final DoublePublisher m_targetAnglePub;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int motorAssembly, double angOffset)
    {
        this(motorAssembly, motorAssembly+ 10, angOffset);
    }
  private MAXSwerveModule(int drivingCANId, int turningCANId, double angOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
    m_angOffset=angOffset;
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
    // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1, 0, 0,0)
            .outputRange(-1, 1);
            // .iZone(1);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    //Type.kDutyCycle

    m_turningConfig.closedLoop.positionWrappingEnabled(true);
    m_turningConfig.closedLoop.positionWrappingMinInput(0);
    m_turningConfig.closedLoop.positionWrappingMaxInput(turningFactor);
     m_turningConfig.encoder.positionConversionFactor(turningFactor/25.0);
   
    m_drivingSparkMax.configure(m_drivingConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
     m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningRelativeEncoder = m_turningSparkMax.getEncoder();
    // m_turningEncoder=(SparkMaxAlternateEncoder) m_turningSparkMax.getAlternateEncoder();
    // m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
      
    m_desiredState.angle = getAbsoluteAngle();
    m_drivingEncoder.setPosition(0);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    var dataTable = inst.getTable("swerveMod");
    var name = "Mod " + turningCANId;
    //m_absoluteEncoderPub = dataTable.getDoubleTopic(name + "/absolute angle").publish();
    m_relativeEncoderPub = dataTable.getDoubleTopic(name + "/relative angle").publish();
    m_targetAnglePub = dataTable.getDoubleTopic(name + "/target angle").publish();
    
  }

  public void dashboardUpdate()
  {
    // m_absoluteEncoderPub.set(m_turningEncoder.getPosition());
    m_relativeEncoderPub.set(m_turningRelativeEncoder.getPosition());
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), getAbsoluteAngle() );
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
        getAbsoluteAngle());
  }
  public double getDriveVelocity()
    {
        return m_drivingEncoder.getVelocity() * kDrivePositionFactor;
    }
  public void setDriveVelocity(double targetSpeed)
    {
        // ToDo: convert target speed to desired rps      
        m_drivingSparkMax.setVoltage(targetSpeed);
        // m_drivingPIDController.setReference(targetSpeed, ControlType.kVelocity);
        // m_drivingPIDController.setReference(targetSpeed * maxSpeedMetersPerSecond, ControlType.kVelocity);
      }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    //SwerveModuleState correctedDesiredState = new SwerveModuleState();
    //correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    //correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    desiredState.optimize(getAbsoluteAngle());
       
    setDriveVelocity(desiredState.speedMetersPerSecond);

    setSteerAngleInRadians(desiredState.angle.getRadians());
    // Optimize the reference state to avoid spinning further than 90 degrees.
    //DEPRECATED
    //SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
       // new Rotation2d(m_turningEncoder.getPosition()));
    //Rotation2d currentRotation =new Rotation2d(m_turningEncoder.getPosition());
    //correctedDesiredState.optimize(currentRotation);
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    //m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    ///m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }
 public void setSteerAngleInRadians(double targetAngleInRadians)
    {
      m_targetAnglePub.set(targetAngleInRadians);
        
      m_turningPIDController.setReference(targetAngleInRadians, SparkMax.ControlType.kPosition);
    }

    /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
  private Rotation2d getAbsoluteEncoder()
    {
        // From ThriftyBot User's Guide: 
        // The signal pin is a 12-bit absolute position reference with the lower bound being the ground pin and the
        // upper bound being the 5V pin. This 5V is relative to the 5V rail on the Roborio, which is often not exactly
        // 5V. This allows for a resolution of 1/4096*360 (~.09) degrees.
        double rotations = m_turningEncoder.getPosition();
        double degrees = rotations * 2*Math.PI;
        if (degrees < 0)
          degrees += 2*Math.PI;    

        return Rotation2d.fromRadians(degrees);
    }
  public void resetAngleEncoderToAbsolute()
    {
        m_turningRelativeEncoder.setPosition(getAbsoluteEncoderPosition().getRadians());
    }
    private Rotation2d getAbsoluteEncoderPosition()
    {
        // convert from absolute Thrifty coder turning counter-clockwise as positive to
        //  relative encoder in assembly turning clockwise as positive

        double startingAngle =m_angOffset  - getAbsoluteEncoder().getRadians();
    
        if (startingAngle < 0)
        {
          startingAngle = startingAngle + (2 * Math.PI);
        }

        return Rotation2d.fromRadians(startingAngle);
    }

  private Rotation2d getAbsoluteAngle()
  {
    // return new Rotation2d(m_turningEncoder.getPosition());
    return new Rotation2d(m_turningEncoder.getPosition());
  }
}
