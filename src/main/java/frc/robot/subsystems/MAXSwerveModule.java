// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj.AnalogEncoder;
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
  //private final AnalogEncoder m_turningAbsoluteEncoder;
  private final SparkRelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  
  //private SparkMaxConfig m_drivingConfig;
  //private SparkMaxConfig m_turningConfig;
  private DigitalInput m_dutyCycleInput;
  private DutyCycle m_dutyCycleEncoder;
  private double m_chassisAngularOffset = 0;
  //private final double m_steerOffset;
  private static final int kEncoderResolution = 4096;
  private static final double kWheelRadius = 2.0 * 0.0254;
  private static final double kGearboxRatio = 1.0 / 6.12;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private static final double kWheelCircumference = 0.3;
  private static final double kDrivePositionFactor = kWheelCircumference * kGearboxRatio;
  private static final SparkMaxConfig m_drivingConfig = new SparkMaxConfig();
  private static final SparkMaxConfig m_turningConfig = new SparkMaxConfig();
  private final double m_steerOffset;
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int motorAssembly, double steerOffset)
    {
        this(motorAssembly, motorAssembly+ 10, steerOffset);
    }
  private MAXSwerveModule(int drivingCANId, int turningCANId, double steerOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_steerOffset=steerOffset;
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
    double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
    //m_steerOffset = steerOffset;
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    //SparkMaxConfig config = new SparkMaxConfig();
    //config.idleMode(SparkBaseConfig.IdleMode.kBrake);
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
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true)
            .voltageCompensation(12.6);
    m_turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
    m_turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            
            .pidf(1.8, 0, 0,0)
            .outputRange(-1, 1)
            .iZone(1);
            
            //.positionWrappingEnabled(true)
            //.positionWrappingInputRange(0, turningFactor);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    //Type.kDutyCycle
    m_turningConfig.closedLoop.positionWrappingEnabled(true);
    m_turningConfig.closedLoop.positionWrappingMinInput(0);
    m_turningConfig.closedLoop.positionWrappingMaxInput(2*Math.PI);
    m_turningConfig.encoder.positionConversionFactor(2*Math.PI/25);
   
    m_drivingSparkMax.configure(m_drivingConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder=(SparkRelativeEncoder) m_turningSparkMax.getEncoder();
    //m_turningAbsoluteEncoder = new AnalogEncoder(turningAbsoluteEncoder);
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
      
    //m_chassisAngularOffset = chassisAngularOffset;
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
  public double getDriveVelocity()
    {
        return m_drivingEncoder.getVelocity() * kDrivePositionFactor;
    }
  public void setDriveVelocity(double targetSpeed)
    {
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
        // ToDo: convert target speed to desired rps      
        m_drivingSparkMax.setVoltage(targetSpeed);
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
    desiredState.optimize(getAbsoluteEncoderPosition());
       
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
    //m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }
 public void setSteerAngleInRadians(double targetAngleInRadians)
    {
        
        m_turningPIDController.setReference(targetAngleInRadians, SparkMax.ControlType.kPosition);
    }
    private Rotation2d getAbsEncoder()
    {
        // From ThriftyBot User's Guide: 
        // The signal pin is a 12-bit absolute position reference with the lower bound being the ground pin and the
        // upper bound being the 5V pin. This 5V is relative to the 5V rail on the Roborio, which is often not exactly
        // 5V. This allows for a resolution of 1/4096*360 (~.09) degrees.
        double rotations = m_turningEncoder.getPosition();
        double degrees = rotations * 360;
        if (degrees < 0)
          degrees += 360;    

        return Rotation2d.fromDegrees(degrees);
    }
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
  private Rotation2d getAbsoluteEncoderPosition()
    {
        double startingAngle = m_steerOffset - getAbsEncoder().getRadians();
    
        if (startingAngle < 0)
        {
          startingAngle = startingAngle + (2 * Math.PI);
        }

        // need to convert from absolute CAN coder turning clockwise as positive to
        //  relative encoder in assembly turning counter-clockwise as positive
        startingAngle = 2 * Math.PI - startingAngle;

        return Rotation2d.fromRadians(startingAngle);
    }
    public void resetAngleEncoderToAbsolute()
    {
        m_turningEncoder.setPosition(getAbsoluteEncoderPosition().getRadians());
    }
}
