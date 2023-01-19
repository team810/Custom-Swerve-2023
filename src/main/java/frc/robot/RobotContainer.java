package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drivetrain.DriveAuto;
import frc.robot.commands.Drivetrain.SwerveDriveKinematics;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain Drive = new Drivetrain();

  private final Joystick right = new Joystick(0);
  private final Joystick left = new Joystick(1);
  private boolean FeildSentric = false;
  private DriveAuto m_DriveAuto;
  private final SwerveDriveKinematics SwerveDrive = new SwerveDriveKinematics(Drive,
          () -> right.getRawAxis(1) * Constants.MaxSpeed,  // Y
          () -> right.getRawAxis(0) * Constants.MaxSpeed, // X
          () -> -left.getRawAxis(0), // Rotation
          () -> true
  );


  public RobotContainer() {

    // Configure the button bindings

    Drive.setDefaultCommand(SwerveDrive);
    m_DriveAuto = new DriveAuto("New Path", () -> SwerveDrive, Drive);
    configureButtonBindings();
    Drive.ResetEncoders();
    Drive.Zero();

  }
  private void configureButtonBindings() {
    Trigger Left_Trigger = new Trigger(() -> left.getTriggerPressed());
    Left_Trigger.onTrue(new InstantCommand(() -> Drive.Zero()));



  }

  public Command getAutonomousCommand() {

//   return m_DriveAuto.Auto_Command();
    return null;
  }
}
