// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.algaeLiftSubsystem;

public class ExtendAlgaeLiftCommand extends Command {
  private algaeLiftSubsystem m_Algae;
  
  double getRightTriggerAxis;
  int convRightTriggerAxis;
  private Supplier<Double> m_liftValue;
  /** Creates a new ShootNote. */
  public ExtendAlgaeLiftCommand(algaeLiftSubsystem algae, Supplier<Double> liftValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
    m_Algae = algae;
    m_liftValue=liftValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    //m_shooter.setShooterSpeed(-1);
    //m_shooter.setShooterSpeed(-1*m_shooter.shootSpeed);
    m_Algae.setLiftSpeed(.30*m_liftValue.get());
    SmartDashboard.putBoolean("outtakeon", true);
    SmartDashboard.putNumber("num", m_Algae.shootSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Algae.setLiftSpeed(.30*m_liftValue.get());
    SmartDashboard.putBoolean("outtakeon", true);
    SmartDashboard.putNumber("num", m_Algae.shootSpeed);
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    /*
    if ((getRightTriggerAxis*100)%4==0){
      convRightTriggerAxis=(int) getRightTriggerAxis*100;
      threshold=convRightTriggerAxis/4;
    }
     */
    
    
      //m_shooter.setFeederSpeed(-1*getRightTriggerAxis);
      
    
    
    //SmartDashboard.putNumber("num", m_shooter.shootSpeed);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    m_Algae.setLiftSpeed(0);
    //SmartDashboard.putBoolean("outtakeon", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
