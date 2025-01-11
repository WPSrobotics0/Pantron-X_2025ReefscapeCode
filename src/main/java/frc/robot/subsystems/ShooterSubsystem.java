// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {

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

  private final CANSparkMax m_leftShooter = new CANSparkMax(SubsystemConstants.kLeftShooterCanId, MotorType.kBrushed);
  private final CANSparkMax m_leftFeeder = new CANSparkMax(SubsystemConstants.kLeftFeederCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightShooter = new CANSparkMax(SubsystemConstants.kRightShooterCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightFeeder = new CANSparkMax(SubsystemConstants.kRightFeederCanId, MotorType.kBrushed);
  public int shootMode=3;
  public double shootSpeed=1.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooter.restoreFactoryDefaults();
    m_leftFeeder.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();
    m_rightFeeder.restoreFactoryDefaults();

    m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_leftFeeder.setIdleMode(IdleMode.kCoast);
    m_rightShooter.setIdleMode(IdleMode.kCoast);
    m_rightFeeder.setIdleMode(IdleMode.kCoast);

    m_rightShooter.follow(m_leftShooter,true);
    m_rightFeeder.follow(m_leftFeeder,true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    m_leftShooter.set(speed);
  }
  public void setFeederSpeed(double speed) {
    m_leftFeeder.set(speed);
  }

  public void shoot(double speed) {
    m_leftShooter.set(speed);
    m_rightShooter.follow(m_leftShooter,true);
  }
  public void feed(double speed){
    m_leftFeeder.set(speed);
    m_rightFeeder.follow(m_leftShooter,true);
  }
}
