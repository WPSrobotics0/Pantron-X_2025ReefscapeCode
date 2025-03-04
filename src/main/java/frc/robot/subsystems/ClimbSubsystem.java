// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
//import java.io.ObjectInputFilter.Config;

//import com.revrobotics.servohub.ServoHub.ResetMode;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_climbMotor1 = new SparkMax(SubsystemConstants.kClimbMotor1CanId, MotorType.kBrushless);
  private final SparkMax m_climbMotor2 = new SparkMax(SubsystemConstants.kClimbMotor2CanId, MotorType.kBrushless);

  private SparkClosedLoopController m_climbPid;
  // private RelativeEncoder m_climbEncoder;

  private static double kClimbP = 0.01;
  private static double kClimbI = 0;
  private static double kClimbD = 0;
  private static double kClimbFF = 0;
  private final double kClimbG=-0.054;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    //SparkMaxConfig config = new SparkMaxConfig();
    //config.signals.primaryEncoderPositionPeriodMs(5);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig =new SparkMaxConfig();
    SparkBaseConfig followerConfig = new SparkMaxConfig();
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .closedLoop.pidf(kClimbP, kClimbI, kClimbD,kClimbFF);
    leaderConfig
        .apply(globalConfig);
    followerConfig
        .apply(globalConfig);
    //m_leftShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_climbMotor1.configure(leaderConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    //config.follow(m_leftShooter,true);
    //m_rightShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_climbMotor2.configure(followerConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

    
    
   

    //m_climbMotor1.setIdleMode(IdleMode.kBrake);
    //m_climbMotor2.setIdleMode(IdleMode.kBrake);
    
    //m_climbMotor2.follow(m_climbMotor1, false);
    


    m_climbPid = m_climbMotor1.getClosedLoopController();

    //m_climbPid.setP(kClimbP);
    //m_climbPid.setI(kClimbI);
    //m_climbPid.setD(kClimbD);
    //m_climbPid.setFF(kClimbFF);

    // m_climbEncoder = m_climbMotor1.getEncoder();

    smartDashboardInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    smartDashboardUpdate();
  }

  public void setExtendSpeed(double speed) {
    double realOutput=speed+kClimbG;
    m_climbMotor1.set(realOutput);
    m_climbMotor2.set(-1*realOutput);
  }

  // public void setRetractSpeed(double speed) {
  //   m_climbMotor1.set(-1 * speed);
  //   m_climbMotor2.set(speed);
  // }

  public void smartDashboardInit()
  {
      // SmartDashboard.putNumber("Climb Position", m_climbEncoder.getPosition());
  
      SmartDashboard.putBoolean("Go To Climb Target", false);

      SmartDashboard.putBoolean("Climb Data Update", false);
      SmartDashboard.putNumber("Climb P", kClimbP);
      SmartDashboard.putNumber("Climb I", kClimbI);
      SmartDashboard.putNumber("Climb D", kClimbD);
      SmartDashboard.putNumber("Climb FF", kClimbFF);
  }
  
  private void smartDashboardUpdate()
  {
      // SmartDashboard.putNumber("Climb Position", m_climbEncoder.getPosition());

      if (SmartDashboard.getBoolean("Climb Data Update", false))
      {
        //m_climbPid.setP(SmartDashboard.getNumber("Climb P", kClimbP));
        //m_climbPid.setI(SmartDashboard.getNumber("Climb I", kClimbI));
        //m_climbPid.setD(SmartDashboard.getNumber("Climb D", kClimbD));
        //m_climbPid.setFF(SmartDashboard.getNumber("Climb FF", kClimbFF));

        SmartDashboard.putBoolean("Climb Data Update", false);
      }

      if (SmartDashboard.getBoolean("\"Go To Climb Target", false))
      {
        goToPosition(SmartDashboard.getNumber("Climb Target Position", 0));
        
        SmartDashboard.putBoolean("\"Go To Climb Target", false);
      }
      SmartDashboard.putNumber("Climb Power", m_climbMotor1.getAppliedOutput());
  }

  public void goToPosition(double position) {
    m_climbPid.setReference(position, ControlType.kPosition);
  }
}
