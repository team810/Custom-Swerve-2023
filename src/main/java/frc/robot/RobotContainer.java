package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drivetrain.SwerveDriveKinematics;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain Drive = new Drivetrain();
  private final Joystick right = new Joystick(0);
  private final Joystick left = new Joystick(1);

  private boolean FeildSentric = true;
  private final SwerveDriveKinematics SwerveDrive = new SwerveDriveKinematics(Drive,
          () -> right.getRawAxis(1) * Constants.MaxSpeed,  // Y
          () -> right.getRawAxis(0) * Constants.MaxSpeed, // X
          () -> -left.getRawAxis(2), // Rotation
          () -> FeildSentric
  );


  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    Drive.setDefaultCommand(SwerveDrive);

  }
  private void configureButtonBindings() {

    Trigger Left_Trigger = new Trigger(left::getTrigger); // I know this will make you guys made ::
    Trigger Right_Trigger = new Trigger(right::getTrigger);

    Right_Trigger.toggleOnTrue(new InstantCommand( () -> Drive.ResetEncoders()));

    // This switches between Field Reltive and Robot Sentric
    Left_Trigger.toggleOnTrue(new SequentialCommandGroup(
            new InstantCommand( () -> FeildSentric = !FeildSentric),
            new InstantCommand( () -> Drive.Zero()),
            new InstantCommand( () -> System.out.println("Filed Orinted: " + FeildSentric))
    ));
  }

  public Command getAutonomousCommand() {

    return null;
  }
}
