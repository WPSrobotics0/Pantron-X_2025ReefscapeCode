// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem;

public class ScoreCoralCommand extends Command {
  private CoralSubsystem m_Coral;
  // private int ticks;
  // private int threshold;
  double getRightTriggerAxis;
  int convRightTriggerAxis;
  /** Creates a new ShootNote. */
  public ScoreCoralCommand(CoralSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
    m_Coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    //m_shooter.setShooterSpeed(-1);
    //m_shooter.setShooterSpeed(-1*m_shooter.shootSpeed);
    //ticks = 0;
    //threshold=25;
    SmartDashboard.putBoolean("outtakeon", true);
    SmartDashboard.putNumber("num", m_Coral.shootSpeed);
    m_Coral.setFeederSpeed(-1*0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    /*
    if ((getRightTriggerAxis*100)%4==0){
      convRightTriggerAxis=(int) getRightTriggerAxis*100;
      threshold=convRightTriggerAxis/4;
    }
     */
    
    
      //m_shooter.setFeederSpeed(-1*getRightTriggerAxis);
      
    
    SmartDashboard.putNumber("num", m_Coral.shootSpeed);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    m_Coral.setFeederSpeed(0);
    SmartDashboard.putBoolean("outtakeon", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
