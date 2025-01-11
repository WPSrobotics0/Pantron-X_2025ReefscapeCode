// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax m_climbMotor1 = new CANSparkMax(SubsystemConstants.kClimbMotor1CanId, MotorType.kBrushed);
  private final CANSparkMax m_climbMotor2 = new CANSparkMax(SubsystemConstants.kClimbMotor2CanId, MotorType.kBrushed);

  private SparkPIDController m_climbPid;
  // private RelativeEncoder m_climbEncoder;

  private static double kClimbP = 0.01;
  private static double kClimbI = 0;
  private static double kClimbD = 0;
  private static double kClimbFF = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_climbMotor1.restoreFactoryDefaults();
    m_climbMotor2.restoreFactoryDefaults();

    m_climbMotor1.setIdleMode(IdleMode.kBrake);
    m_climbMotor2.setIdleMode(IdleMode.kBrake);

    m_climbMotor2.follow(m_climbMotor1, false);



    m_climbPid = m_climbMotor1.getPIDController();

    m_climbPid.setP(kClimbP);
    m_climbPid.setI(kClimbI);
    m_climbPid.setD(kClimbD);
    m_climbPid.setFF(kClimbFF);

    // m_climbEncoder = m_climbMotor1.getEncoder();

    smartDashboardInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    smartDashboardUpdate();
  }

  public void setExtendSpeed(double speed) {
    m_climbMotor1.set(speed);
  }

  public void setRetractSpeed(double speed) {
    m_climbMotor1.set(-1 * speed);
  }

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
        m_climbPid.setP(SmartDashboard.getNumber("Climb P", kClimbP));
        m_climbPid.setI(SmartDashboard.getNumber("Climb I", kClimbI));
        m_climbPid.setD(SmartDashboard.getNumber("Climb D", kClimbD));
        m_climbPid.setFF(SmartDashboard.getNumber("Climb FF", kClimbFF));

        SmartDashboard.putBoolean("Climb Data Update", false);
      }

      if (SmartDashboard.getBoolean("\"Go To Climb Target", false))
      {
        goToPosition(SmartDashboard.getNumber("Climb Target Position", 0));
        
        SmartDashboard.putBoolean("\"Go To Climb Target", false);
      }
  }

  public void goToPosition(double position) {
    m_climbPid.setReference(position, ControlType.kPosition);
  }
}
