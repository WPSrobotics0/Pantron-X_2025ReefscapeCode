// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;


public class ClimbExtendCommand extends Command {
  private ClimbSubsystem m_climb;
  /** Creates a new ClimbCommand. */
  public ClimbExtendCommand(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    m_climb = climb;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.setExtendSpeed(0.15);
    SmartDashboard.putBoolean("climbExtendon", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //why different speeds???
    m_climb.setExtendSpeed(0.35);
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
