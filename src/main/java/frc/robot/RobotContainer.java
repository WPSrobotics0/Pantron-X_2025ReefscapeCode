// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.ClimbExtendCommand;

// import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.ExtendAlgaeLiftCommand;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.AutoAlgaeLiftCommand;
import frc.robot.commands.AutoClimbExtendCommand;
// import frc.robot.commands.RetractAlgaeLiftCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.ShootCoralCommand;
import frc.robot.commands.holdAlgaeLiftCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.algaeSubsystem;
import frc.robot.subsystems.algaeLiftSubsystem;
import frc.robot.commands.ShootAlgaeCommand;
import frc.robot.commands.IntakeAlgaeCommand;
// import frc.robot.commands.TeleOpDriveCommand;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;

//import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//FOR TEMPLATE
// import edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final algaeSubsystem m_AlgaeSubsystem = new algaeSubsystem();
  private final algaeLiftSubsystem m_AlgaeLiftSubsystem = new algaeLiftSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeCoralCommand m_IntakeCoralCommand = new IntakeCoralCommand(m_CoralSubsystem);
  private final ShootCoralCommand m_ShootCoralCommand = new ShootCoralCommand(m_CoralSubsystem);
  private final ScoreCoralCommand m_ScoreCoralCommand = new ScoreCoralCommand(m_CoralSubsystem);
  private final ClimbExtendCommand m_ClimbExtendCommand = new ClimbExtendCommand(m_ClimbSubsystem, ()->m_armController.getLeftY(), ()->m_armController.getRightTriggerAxis(),()->m_armController.getLeftTriggerAxis());
  
  // private final ClimbRetractCommand m_ClimbRetractCommand = new ClimbRetractCommand(m_ClimbSubsystem);
  //private final AutoClimbExtendCommand m_AutoClimbExtendCommand=new AutoClimbExtendCommand(m_ClimbSubsystem);
  private final IntakeAlgaeCommand m_IntakeAlgaeCommand = new IntakeAlgaeCommand(m_AlgaeSubsystem);
  private final ShootAlgaeCommand m_ShootAlgaeCommand = new ShootAlgaeCommand(m_AlgaeSubsystem);
  private final ExtendAlgaeLiftCommand m_ExtendAlgaeLiftCommand = new ExtendAlgaeLiftCommand(m_AlgaeLiftSubsystem, ()->m_armController.getRightY());
  //private final RetractAlgaeLiftCommand m_RetractAlgaeLiftCommand=new RetractAlgaeLiftCommand(m_AlgaeSubsystem);
  public final ChassisSpeeds speeds= new ChassisSpeeds(0.0, 0.0, 0);
  private final Trigger m_CoralSensor=new Trigger(()->m_CoralSubsystem.getSensorInput());
  private final SendableChooser<Command> autoChooser;
 
  public final static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort0);

  public final static CommandXboxController m_armController = new CommandXboxController(
      OIConstants.kDriverControllerPort1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer(){//Robot robot) {
    // Configure the trigger bindings
    //m_robot=robot;
  //change if in comp
  //boolean isCompetition = false;
  NetworkTableInstance.getDefault().getTable("limeLight").getEntry("ledMode").setNumber(1);
    NamedCommands.registerCommand("shootAlgae", shootAlgae());
    NamedCommands.registerCommand("intakeAlgae", intakeAlgae());
    NamedCommands.registerCommand("shootCoral", shootCoral());
    NamedCommands.registerCommand("intakeCoral", intakeCoral());
    NamedCommands.registerCommand("algaeLiftExtend", algaeLiftExtend());
    NamedCommands.registerCommand("holdAlgaeLift", holdAlgaeLift());
    NamedCommands.registerCommand("scoreCoral", scoreCoral());
    NamedCommands.registerCommand("LTwoClimb", LTwoClimb());
    NamedCommands.registerCommand("LTwoHold", LTwoHold());
    NamedCommands.registerCommand("LTwoLower", LTwoLower());
    // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    //  (stream) -> isCompetition
    // ? stream.filter(auto -> auto.getName().startsWith("comp"))
    // : stream);
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    /*PREVIOUS CODE
    m_driveXConditioning.setDeadband(0.15);
    m_driveXConditioning.setExponent(2);
    m_driveYConditioning.setDeadband(0.15);
    m_driveYConditioning.setExponent(2);
    m_turnConditioning.setDeadband(0.2);
    m_turnConditioning.setExponent(1.4);
    */
    configureBindings();
    
    //TEMPLATE CODE
    m_DriveSubsystem.setDefaultCommand(new RunCommand(
      () -> m_DriveSubsystem.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        false),
      m_DriveSubsystem));

    // Configure default commands\
    //UNCOMMENT LATER
    //m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        //new RunCommand(
          
            //() -> m_robotDrive.drive(driveControl(),
            //    fieldRelative),
            //m_robotDrive));
    
    
  }
  

  boolean fieldRelative = true;

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    //PREVIOUS CODE
    /*m_DriveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_DriveSubsystem, 
    () -> -m_driverController.getLeftY(), 
    () -> -m_driverController.getLeftX(), 
    () -> m_driverController.getRightX(), () -> true));
    */
    //m_armController.rightTrigger(0.15).whileTrue(m_RetractAlgaeLiftCommand);
    //m_armController.leftTrigger(0.15).whileTrue(m_ExtendAlgaeLiftCommand);

    // m_driverController.y().onTrue(new InstantCommand(() -> fieldRelative = false));
    // m_driverController.x().onTrue(new InstantCommand(() -> fieldRelative = true));

    m_driverController.start().onTrue(new
      InstantCommand(()->m_DriveSubsystem.zeroHeading()));
    

    // you would want these uncomented if you want a working lift
    //m_armController.rightBumper().whileTrue(m_ClimbExtendCommand);
    m_ClimbSubsystem.setDefaultCommand(m_ClimbExtendCommand);
    m_AlgaeLiftSubsystem.setDefaultCommand(m_ExtendAlgaeLiftCommand);
    
    m_armController.getRightTriggerAxis();    
    m_armController.a().whileTrue(m_IntakeAlgaeCommand);
    m_armController.b().whileTrue(m_ShootAlgaeCommand);

    m_armController.x().and(m_CoralSensor.debounce(0.25).negate()).whileTrue(m_IntakeCoralCommand);
    m_armController.start().whileTrue(m_ShootCoralCommand);
    m_armController.y().whileTrue(m_ScoreCoralCommand);
    //m_driverController.start().onTrue(new
    //  InstantCommand(()->m_DriveSubsystem.zeroHeading()));
    m_driverController.y().onTrue(new InstantCommand(() -> fieldRelative = false));
   
  }

  //SendableChooser<Command> chooser;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }
  
  public Command shootAlgae() {
    // An example command will be run in autonomous
    return new ShootAlgaeCommand(m_AlgaeSubsystem).withTimeout(1.5);
  }
  public Command intakeAlgae() {
    // An example command will be run in autonomous
    return new IntakeAlgaeCommand(m_AlgaeSubsystem).withTimeout(.2);
  }
  public Command shootCoral() {
    // An example command will be run in autonomous
    return new ShootCoralCommand(m_CoralSubsystem).withTimeout(1.5);
  }
  public Command intakeCoral() {
    // An example command will be run in autonomous
    return new IntakeCoralCommand(m_CoralSubsystem).withTimeout(1.5);
  }

  //numbers need tweaking, logic should be good though
  public Command algaeLiftExtend(){
    return new AutoAlgaeLiftCommand(m_AlgaeLiftSubsystem,true).withTimeout(1.2);
 }
 public Command holdAlgaeLift(){
  return new AutoAlgaeLiftCommand(m_AlgaeLiftSubsystem,false).withTimeout(.5).andThen(new holdAlgaeLiftCommand(m_AlgaeLiftSubsystem).withTimeout(2.0));
}
public Command scoreCoral(){
  return new ScoreCoralCommand(m_CoralSubsystem).withTimeout(.5);
}
public Command LTwoLower(){
  return new AutoClimbExtendCommand(m_ClimbSubsystem, -1).withTimeout(1);
}
public Command LTwoClimb(){
  return new AutoClimbExtendCommand(m_ClimbSubsystem, 1).withTimeout(1);
}
public Command LTwoHold(){
  return new AutoClimbExtendCommand(m_ClimbSubsystem, 0).withTimeout(2);
}
}