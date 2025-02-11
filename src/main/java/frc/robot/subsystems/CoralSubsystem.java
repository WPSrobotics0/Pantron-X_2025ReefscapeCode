// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class CoralSubsystem extends SubsystemBase {

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
  private final SparkMax m_leftFeeder = new SparkMax(SubsystemConstants.kLeftShooterCanId, MotorType.kBrushed);
  //private final SparkMax m_rightShooter = new SparkMax(SubsystemConstants.kRightShooterCanId, MotorType.kBrushed);
  private final SparkMax m_rightFeeder = new SparkMax(SubsystemConstants.kRightShooterCanId, MotorType.kBrushed);
  public int shootMode=3;
  public double shootSpeed=1.0;

  /** Creates a new ShooterSubsystem. */
  public CoralSubsystem() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig =new SparkMaxConfig();
    SparkBaseConfig followerConfig = new SparkMaxConfig();
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    leaderConfig
        .apply(globalConfig);
    followerConfig
        .apply(globalConfig);
    //m_leftShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_leftFeeder.configure(leaderConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    //config.follow(m_leftShooter,true);
    //m_rightShooter.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
    m_rightFeeder.configure(followerConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

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
    m_rightFeeder.set(-1*speed);
  }

  //public void shoot(double speed) {
    //m_leftShooter.set(speed);
    //idk why this is here tbh
    //m_rightShooter.follow(m_leftShooter,true);
  //}
  public void feed(double speed){
    m_leftFeeder.set(speed);
    m_rightFeeder.set(-1*speed);

    //m_rightFeeder.follow(m_leftShooter,true);
  }
}
