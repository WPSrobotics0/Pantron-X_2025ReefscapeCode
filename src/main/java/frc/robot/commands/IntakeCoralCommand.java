// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class IntakeCoralCommand extends Command {
  private CoralSubsystem m_Coral;
  /** Creates a new IntakeNoteCommand. */
  public IntakeCoralCommand(CoralSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
    m_Coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_shooter.setShooterSpeed(0.5);
    m_Coral.setFeederSpeed(-1*.5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    m_Coral.setFeederSpeed(0);
    

  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
