// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.algaeSubsystem;

public class ShootAlgaeCommand extends Command {
  private algaeSubsystem m_shooter;
  private int ticks;
  private int threshold;
  double getRightTriggerAxis;
  int convRightTriggerAxis;
  /** Creates a new ShootNote. */
  public ShootAlgaeCommand(algaeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    //m_shooter.setShooterSpeed(-1);
    //m_shooter.setShooterSpeed(-1*m_shooter.shootSpeed);
    ticks = 0;
    threshold=25;
    //SmartDashboard.putBoolean("outtakeon", true);
    //SmartDashboard.putNumber("num", m_shooter.shootSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    ticks +=1;
    /*
    if ((getRightTriggerAxis*100)%4==0){
      convRightTriggerAxis=(int) getRightTriggerAxis*100;
      threshold=convRightTriggerAxis/4;
    }
     */
    
    if (ticks == threshold) {
      //m_shooter.setFeederSpeed(-1*getRightTriggerAxis);
      m_shooter.setFeederSpeed(-1*m_shooter.shootSpeed);
    }
    
    //SmartDashboard.putNumber("num", m_shooter.shootSpeed);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    m_shooter.setFeederSpeed(0);
    //SmartDashboard.putBoolean("outtakeon", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
