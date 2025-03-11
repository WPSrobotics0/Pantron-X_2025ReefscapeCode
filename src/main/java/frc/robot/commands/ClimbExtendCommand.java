// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbExtendCommand extends Command {
  private ClimbSubsystem m_climb;
  private Supplier<Double> m_climbValue;
  private Supplier<Double> m_rightValue;
  private Supplier<Double> m_leftValue;

  /** Creates a new ClimbCommand. */
  public ClimbExtendCommand(ClimbSubsystem climb, Supplier<Double> climbValue, Supplier<Double> rightValue, Supplier<Double> leftValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climb = climb;
    m_climbValue = climbValue;
    m_leftValue=leftValue;
    m_rightValue=rightValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putBoolean("climbExtendon", true);
    if (m_leftValue.get()>=0.1){
      m_climb.setExtendSpeed(0.5*m_leftValue.get());
    }
    else if (m_rightValue.get()>=0.1){
      m_climb.setExtendSpeed(-0.4*m_rightValue.get());
    }
    else if (m_climbValue.get()>=0.05 || m_climbValue.get()<=-0.05){
      m_climb.setExtendSpeed(0.1*m_climbValue.get());
    }
    else{
      m_climb.setExtendSpeed(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // why different speeds???
    if (m_leftValue.get()>=0.1){
      m_climb.setExtendSpeed(0.5*m_leftValue.get());
    }
    else if (m_rightValue.get()>=0.1){
      m_climb.setExtendSpeed(-0.4*m_rightValue.get());
    }
    else if (m_climbValue.get()>=0.05 || m_climbValue.get()<=-0.05){
      m_climb.setExtendSpeed(0.1*m_climbValue.get());
    }
    else{
      m_climb.setExtendSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setExtendSpeed(0);
    SmartDashboard.putBoolean("climbExtendon", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
