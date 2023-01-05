// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;
  
  //NavX
  private final AHRS gyro;// change to I2C if not working
  
  //Odometry
  SwerveDriveOdometry odometry;

  public Drivetrain() {
    frontRight = new SwerveModule(this, 
      Constants.FRONT_RIGHT_CAN, 
      Constants.FRONT_RIGHT, 
      Constants.FRONT_RIGHT_PORT_1, 
      Constants.FRONT_RIGHT_PORT_2, false, false);
    
    frontLeft = new SwerveModule(this,
      Constants.FRONT_LEFT_CAN, 
      Constants.FRONT_LEFT, 
      Constants.FRONT_LEFT_PORT_1, 
      Constants.FRONT_LEFT_PORT_2,  false, false);

    backRight = new SwerveModule(this,
      Constants.BACK_RIGHT_CAN,
      Constants.BACK_RIGHT,
      Constants.BACK_RIGHT_PORT_1,
      Constants.BACK_RIGHT_PORT_2, false, false);

    backLeft = new SwerveModule(this,
      Constants.BACK_LEFT_CAN,
      Constants.BACK_LEFT, 
      Constants.BACK_LEFT_PORT_1, 
      Constants.BACK_LEFT_PORT_2,  false, false);

      gyro = new AHRS(SPI.Port.kMXP); 
      gyro.reset();

      odometry = new SwerveDriveOdometry(Constants.m_kinematics, gyro.getRotation2d());

      resetOdometry(getPose());
  }
  
  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
      return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
      return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);

    // SASHA'S NOTES: the module states that are passed here are basically raw joystick values
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*odometry.update(getRotation2d(), frontLeft.getState().angle.getRadians(), frontRight.getState().angle.getRadians(), backLeft.getState().angle.getRadians(),
    backRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation());

    SmartDashboard.putNumber("Back Right Speed", backRight.getDriveVelocity());
    SmartDashboard.putNumber("Back Left Speed", backLeft.getDriveVelocity());
    SmartDashboard.putNumber("Front Right Speed", frontRight.getDriveVelocity());
    SmartDashboard.putNumber("Front left Speed", frontLeft.getDriveVelocity());

    SmartDashboard.putNumber("Back Right Encoder position", backRight.getTurningPosition());
    SmartDashboard.putNumber("Back left Encoder position", backLeft.getTurningPosition());
    SmartDashboard.putNumber("Front Right Encoder position", frontRight.getTurningPosition());
    SmartDashboard.putNumber("Front left Encoder position", frontLeft.getTurningPosition());

   SmartDashboard.putNumber("Back Right Encoder target angle", backRight.getAngle());
    SmartDashboard.putNumber("Back left Encoder target angle", backLeft.getAngle());
    SmartDashboard.putNumber("Front Right Encoder target angle", frontRight.getAngle());
    SmartDashboard.putNumber("Front left Encoder target angle", frontLeft.getAngle());

    SmartDashboard.putNumber("Back Right target state", backRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Back left target state", backLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Front Right target state", frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Front left target state", frontLeft.getState().angle.getRadians());*/
  }
}
