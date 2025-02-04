// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class algaeSubsystem extends SubsystemBase {

  public enum ShooterLocation
  {
    Generic,
    Trap,
    CenterSpeaker,
    LeftSpeaker,
    RightSpeaker,
    Amp,
    SourceLoad,
  };

  //private final SparkMax m_leftShooter = new SparkMax(SubsystemConstants.kLeftShooterCanId, MotorType.kBrushed);
  private final SparkMax m_leftFeeder = new SparkMax(SubsystemConstants.kLeftAlgaeCanId, MotorType.kBrushless);
  //private final SparkMax m_rightShooter = new SparkMax(SubsystemConstants.kRightShooterCanId, MotorType.kBrushed);
  private final SparkMax m_rightFeeder = new SparkMax(SubsystemConstants.kRightAlgaeCanId, MotorType.kBrushless);
  public int shootMode=3;
  public double shootSpeed=0.1;
  //private SparkRelativeEncoder m_algaeLeftEncoder;
  //private double m_clawIntakeSpeed = 0.2;
  //private double m_clawShootSpeed = 0.2;
  /** Creates a new ShooterSubsystem. */
  public algaeSubsystem() {
    SparkMaxConfig configLeft = new SparkMaxConfig();
    SparkMaxConfig configRight= new SparkMaxConfig();
    configLeft.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //m_leftShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_leftFeeder.configure(configLeft,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    //config.follow(m_leftShooter,true);
    //m_rightShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    //config.follow(m_leftFeeder,true);
    configRight.idleMode(SparkBaseConfig.IdleMode.kBrake);

    m_rightFeeder.configure(configRight,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    SmartDashboard.putNumber("dsd", 1);
    //m_algaeLeftEncoder=(SparkRelativeEncoder) m_leftFeeder.getEncoder();
    /*m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_leftFeeder.setIdleMode(IdleMode.kCoast);
    m_rightShooter.setIdleMode(IdleMode.kCoast);
    m_rightFeeder.setIdleMode(IdleMode.kCoast);*/

    //m_rightShooter.follow(m_leftShooter,true);
    //m_rightFeeder.follow(m_leftFeeder,true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  //public void setShooterSpeed(double speed) {
    //m_leftShooter.set(speed);
  //}
  public void setFeederSpeed(double speed) {
    m_leftFeeder.set(speed);
    m_rightFeeder.set(speed);
    SmartDashboard.putBoolean("works", true);
  }

  //public void shoot(double speed) {
    //m_leftShooter.set(speed);
    //idk why this is here tbh
    //m_rightShooter.follow(m_leftShooter,true);
  //}
  public void feed(double speed){
    m_leftFeeder.set(speed);
    m_rightFeeder.set(speed);
    SmartDashboard.putBoolean("works", true);
    //m_rightFeeder.follow(m_leftShooter,true);
  }
}
