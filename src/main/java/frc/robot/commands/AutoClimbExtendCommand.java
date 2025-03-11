// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;


public class AutoClimbExtendCommand extends Command {
  private ClimbSubsystem m_climb;
  private double m_climbValue;
  /** Creates a new ClimbCommand. */
  public AutoClimbExtendCommand(ClimbSubsystem climb, double climbValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climb = climb;
    m_climbValue=climbValue;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_climb.setRetractSpeed(0.1);
    m_climb.setExtendSpeed(-0.22*m_climbValue);
    SmartDashboard.putBoolean("climbRetracton", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_climb.setRetractSpeed(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_climb.setExtendSpeed(0);
    SmartDashboard.putBoolean("climbRetracton", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
