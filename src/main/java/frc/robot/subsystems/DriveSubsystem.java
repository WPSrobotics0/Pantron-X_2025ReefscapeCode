  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//COMPLETELY BROKEN UNTIL GYRO SET UP

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  
  public static final double kMaxSpeed = 3.0; 
  public static final double kMaxAngularSpeed = Math.PI;
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId);
  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(DriveConstants.kFrontRightDrivingCanId);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(DriveConstants.kRearRightDrivingCanId);

  private SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    getAngle(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    zeroHeading();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_Odometry.update(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    m_rearRight.dashboardUpdate();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void resetPose(Pose2d pose) {
    m_Odometry.resetPosition(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    double ySpeed=speeds.vyMetersPerSecond *DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeed=speeds.vxMetersPerSecond *DriveConstants.kMaxSpeedMetersPerSecond;
    double rot = speeds.omegaRadiansPerSecond *DriveSubsystem.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
       swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
//resetSteeringMotorsToAbsolute(); ADD THIS
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */

  /** Zeroes the heading of the robot. */

  
  public void zeroHeading() {
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */

  //public double getHeading() {
  //  return getAngle().getDegrees();
  //}
  private void resetSteeringMotorsToAbsolute()
    {
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_rearLeft.resetAngleEncoderToAbsolute();
        m_rearRight.resetAngleEncoderToAbsolute();
    }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(0);
  }
}
