// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.networktables.DoublePublisher;
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
    SmartDashboard.putNumber("Controll y power", m_yJoystickSupplier.get());
    if(m_isTeleopEnabled.get())
      setMotorSpeeds();
  }

  public void setMotorSpeeds(){
    var yJoystickSpeed = m_yJoystickSupplier.get();
    if (Math.abs(yJoystickSpeed) < OIConstants.kDriveDeadband)
    {
      yJoystickSpeed = 0.0;
    }

    var xJoystickSpeed = m_xJoystickSupplier.get();
    if (Math.abs(xJoystickSpeed) < OIConstants.kDriveDeadband)
    {
      xJoystickSpeed = 0.0;
    }

    var rotJoystickSpeed = m_turnJoystickSupplier.get();
    if (Math.abs(rotJoystickSpeed) < OIConstants.kDriveDeadband)
    {
      rotJoystickSpeed = 0.0;
    }

    SmartDashboard.putNumber("dtxSpeed", xJoystickSpeed);
    SmartDashboard.putNumber("dtySpeed", yJoystickSpeed);
    SmartDashboard.putNumber("dtaSpeed", rotJoystickSpeed);
    m_Drive.drive(yJoystickSpeed, xJoystickSpeed, rotJoystickSpeed, false);
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
