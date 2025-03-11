// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.function.Supplier;

// import java.util.function.Supplier;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeLiftSubsystem;

public class AutoAlgaeLiftCommand extends Command {
  private algaeLiftSubsystem m_Algae;
    private Boolean m_isRetract;

  /** Creates a new IntakeNoteCommand. */
  public AutoAlgaeLiftCommand(algaeLiftSubsystem algae, Boolean isRetract) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
    m_Algae = algae;
    m_isRetract=isRetract;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_shooter.setShooterSpeed(0.5);
    SmartDashboard.putBoolean("autoAlgaeWork", true);
    double speed=.1;
    if (m_isRetract==false){
        m_Algae.setLiftSpeed(speed*-1);
    }
    else{
      m_Algae.setLiftSpeed(speed);
    }
    //SmartDashboard.putBoolean("intakeon", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    m_Algae.setLiftSpeed(0);
    //    SmartDashboard.putBoolean("intakeon", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
