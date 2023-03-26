package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DrivetrainCommand extends CommandBase {

  public final Drivetrain swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public DrivetrainCommand(Drivetrain swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction)
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
  public void execute()
  {
    Drive(xSpdFunction.get(), ySpdFunction.get(), turningSpdFunction.get(), fieldOrientedFunction.get());
  }
  public void Drive(double X_Speed, double Y_Speed, double TuringSpeed, boolean FeildCentric) {
    // 1. Get real-time joystick inputs
    double xSpeed = X_Speed;
    double ySpeed = Y_Speed;
    double turningSpeed = TuringSpeed;

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0.0;

//    // // 3. Make the driving smoother
     xSpeed = xLimiter.calculate(xSpeed);
     ySpeed = yLimiter.calculate(ySpeed);
     turningSpeed = turningLimiter.calculate(turningSpeed);
    //     * Constants.kMaxAngularSpeedRadiansPerSecondSquared;

    // 4. Construct desired chassis speeds

    ChassisSpeeds chassisSpeeds;
    if (FeildCentric) { // Feild Reltive
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else { // Robot centric
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
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