package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

public class SwerveJoystick extends CommandBase {

  private final Drivetrain swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystick(Drivetrain swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) 
  {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
    this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
    this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularSpeedRadiansPerSecondSquared);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // // 3. Make the driving smoother
    // xSpeed = xLimiter.calculate(xSpeed) * Constants.kMaxSpeedMetersPerSecond;
    // ySpeed = yLimiter.calculate(ySpeed) * Constants.kMaxSpeedMetersPerSecond;
    // turningSpeed = turningLimiter.calculate(turningSpeed)
    //     * Constants.kMaxAngularSpeedRadiansPerSecondSquared;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // SASHA'S NOTES: I used the ySpeed for the speed of all 4 wheels, and the "turningSpeed" as the POSITION of all wheels
    //                meaning that the "turningSpeed" variable is misslabled and should be "turningPosition"
    //                This also means that all 4 wheels should turn in the same direction and at the same speed, allowing
    //                us to debug our setup and make sure we put everything in the right direction
    // SwerveModuleState[] moduleStates = 
    // {
    //   new SwerveModuleState(ySpeed, new Rotation2d(Math.PI * turningSpeed)),
    //   new SwerveModuleState(ySpeed, new Rotation2d(Math.PI * turningSpeed)),
    //   new SwerveModuleState(ySpeed, new Rotation2d(Math.PI * turningSpeed)),
    //   new SwerveModuleState(ySpeed, new Rotation2d(Math.PI * turningSpeed))
    // };

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}