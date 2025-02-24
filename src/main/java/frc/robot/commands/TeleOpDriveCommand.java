// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.OIConstants;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends Command {
  private DriveSubsystem m_Drive;
  
  double getRightTriggerAxis;
  private Supplier<Double> m_xJoystickSupplier;
  private Supplier<Double> m_yJoystickSupplier;
  private Supplier<Double> m_turnJoystickSupplier;
  private Supplier<Boolean> m_isTeleopEnabled;
  int convRightTriggerAxis;
  /** Creates a new ShootNote. */
  public TeleOpDriveCommand(DriveSubsystem drive,Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> turnJoystick,
  Supplier<Boolean> isTeleopEnabled) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_xJoystickSupplier = xJoystick;
    m_yJoystickSupplier = yJoystick;
    m_turnJoystickSupplier = turnJoystick;
    m_isTeleopEnabled = isTeleopEnabled;
    m_Drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //getRightTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    //m_shooter.setShooterSpeed(-1);
    //m_shooter.setShooterSpeed(-1*m_shooter.shootSpeed);
    //ticks = 0;
    //threshold=25;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_isTeleopEnabled.get())
      m_Drive.drive(getMotorSpeeds(), false);
    
    
  }
  public ChassisSpeeds getMotorSpeeds(){
    ChassisSpeeds speeds=new ChassisSpeeds();
    speeds.vyMetersPerSecond=-MathUtil.applyDeadband(m_xJoystickSupplier.get(), OIConstants.kDriveDeadband);
    speeds.vxMetersPerSecond=-MathUtil.applyDeadband(m_yJoystickSupplier.get(), OIConstants.kDriveDeadband);
    speeds.omegaRadiansPerSecond=MathUtil.applyDeadband(m_turnJoystickSupplier.get(), OIConstants.kDriveDeadband);
    SmartDashboard.putNumber("dtxSpeed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("dtySpeed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("dtaSpeed", speeds.omegaRadiansPerSecond);
    return speeds;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setShooterSpeed(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
