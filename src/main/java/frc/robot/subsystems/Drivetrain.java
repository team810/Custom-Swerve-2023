// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;

  public SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
  //NavX
  private final AHRS gyro;// change to I2C if not working

  //Odometry
  SwerveDriveOdometry odometry;


  public Drivetrain() {
    frontRight = new SwerveModule(
      Constants.FRONT_RIGHT_CAN,
      Constants.FRONT_RIGHT,
      Constants.FRONT_RIGHT_PORT_1,
      Constants.FRONT_RIGHT_PORT_2, false, false,
    Constants.kEncoderResolution);

    frontLeft = new SwerveModule(
      Constants.FRONT_LEFT_CAN,
      Constants.FRONT_LEFT,
      Constants.FRONT_LEFT_PORT_1,
      Constants.FRONT_LEFT_PORT_2,  false, false, 1105);

    backRight = new SwerveModule(
      Constants.BACK_RIGHT_CAN,
      Constants.BACK_RIGHT,
      Constants.BACK_RIGHT_PORT_1,
      Constants.BACK_RIGHT_PORT_2, false, false,
            Constants.kEncoderResolution);

    backLeft = new SwerveModule(
      Constants.BACK_LEFT_CAN,
      Constants.BACK_LEFT,
      Constants.BACK_LEFT_PORT_1,
      Constants.BACK_LEFT_PORT_2,  false, false,
            Constants.kEncoderResolution);

      gyro = new AHRS(SPI.Port.kMXP);
      gyro.reset();

      modulePositions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(),frontLeft.getAngle());
      modulePositions[1] = new SwerveModulePosition(frontRight.getDrivePosition(),frontRight.getAngle());
      modulePositions[2] = new SwerveModulePosition(backLeft.getDrivePosition(),backLeft.getAngle());
      modulePositions[3] = new SwerveModulePosition(backRight.getDrivePosition(),backRight.getAngle());

      odometry = new SwerveDriveOdometry(Constants.m_kinematics,gyro.getRotation2d(), modulePositions);
      ResetEncoders();
      resetOdometry(getPose());
      Unlock();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
      return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
      //return Rotation2d.fromDegrees(getHeading());
      return gyro.getRotation2d();
  }

  public Pose2d getPose() {
      return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {

    //odometry.resetPosition(pose, modulePositions ,getRotation2d());
    odometry.resetPosition(getRotation2d(), modulePositions, pose);
  }
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);

        modulePositions[0] = new SwerveModulePosition(desiredStates[0].speedMetersPerSecond, desiredStates[0].angle);
        modulePositions[1] = new SwerveModulePosition(desiredStates[1].speedMetersPerSecond, desiredStates[1].angle);
        modulePositions[2] = new SwerveModulePosition(desiredStates[2].speedMetersPerSecond, desiredStates[2].angle);
        modulePositions[3] = new SwerveModulePosition(desiredStates[3].speedMetersPerSecond, desiredStates[3].angle);
        // SASHA'S NOTES: the module states that are passed here are basically raw joystick values
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public void Zero()
    {
        gyro.zeroYaw();
    }

    public void Lock_Motor()
    {
        frontRight.Lock();
        frontLeft.Lock();
        backLeft.Lock();
        backRight.Lock();
    }
    public void Unlock()
    {
        frontLeft.Unlock();
        frontRight.Unlock();
        backLeft.Unlock();
        backRight.Unlock();
    }
    public void ResetEncoders()
    {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public Pose2d getpos()
    {
        return odometry.getPoseMeters();
    }
  @Override
  public void periodic() {
      odometry.update(gyro.getRotation2d(), modulePositions);

      //System.out.println(frontLeft.GetEncoder().get());

//    SmartDashboard.putNumber("Back Right Speed", backRight.getDriveVelocity());
//    SmartDashboard.putNumber("Back Left Speed", backLeft.getDriveVelocity());
//    SmartDashboard.putNumber("Front Right Speed", frontRight.getDriveVelocity());
//    SmartDashboard.putNumber("Front left Speed", frontLeft.getDriveVelocity());
//
//    SmartDashboard.putNumber("Back Right Encoder position", backRight.getTurningPosition());
//    SmartDashboard.putNumber("Back left Encoder position", backLeft.getTurningPosition());
//    SmartDashboard.putNumber("Front Right Encoder position", frontRight.getTurningPosition());
//    SmartDashboard.putNumber("Front left Encoder position", frontLeft.getTurningPosition());
////
////    SmartDashboard.putNumber("Back Right Encoder target angle", backRight.getAngle());
////    SmartDashboard.putNumber("Back left Encoder target angle", backLeft.getAngle());
////    SmartDashboard.putNumber("Front Right Encoder target angle", frontRight.getAngle());
////    SmartDashboard.putNumber("Front left Encoder target angle", frontLeft.getAngle());
//
//    SmartDashboard.putNumber("Back Right target state", backRight.getState().angle.getRadians());
//    SmartDashboard.putNumber("Back left target state", backLeft.getState().angle.getRadians());
//    SmartDashboard.putNumber("Front Right target state", frontRight.getState().angle.getRadians());
//    SmartDashboard.putNumber("Front left target state", frontLeft.getState().angle.getRadians());

  }
}
