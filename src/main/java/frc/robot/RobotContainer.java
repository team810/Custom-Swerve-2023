package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drivetrain.SwerveDriveKinematics;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain Drive = new Drivetrain();
  private final Joystick right = new Joystick(0);
  private final Joystick left = new Joystick(1);

  private final SwerveDriveKinematics SwerveDrive = new SwerveDriveKinematics(Drive,
          () -> right.getRawAxis(1) * Constants.MaxSpeed,  // Y
          () -> right.getRawAxis(0) * Constants.MaxSpeed, // X
          () -> -left.getRawAxis(2), // Rotation
          () -> true
  );


  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    Drive.setDefaultCommand(SwerveDrive);

  }
  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {

    return null;
  }
}
