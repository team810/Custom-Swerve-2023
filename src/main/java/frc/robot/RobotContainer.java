package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.Drivetrain;


public class RobotContainer {
//  private final Joystick right = new Joystick(0);
//  private final Joystick left = new Joystick(1);
  private final XboxController xboxController = new XboxController(0);

  private final Drivetrain m_drivetrain = new Drivetrain();

  public RobotContainer() {

    m_drivetrain.setDefaultCommand(new DrivetrainCommand(
            m_drivetrain,
            () -> Math.pow(xboxController.getLeftY(), 3) * Constants.MaxSpeed, // Y
            () -> Math.pow(xboxController.getLeftX(), 3) * Constants.MaxSpeed, // X
            () -> - Math.pow(xboxController.getRightX(), 3) * Constants.MaxTurnSpeed, // Z
            () -> true
    ));
    configureButtonBindings();
  }
  private void configureButtonBindings() {
    Trigger left_Trigger = new Trigger(xboxController::getAButton);
    Trigger right_Trigger = new Trigger(xboxController::getBButton);



    left_Trigger.whileTrue(new InstantCommand(m_drivetrain::lock_motor));
    left_Trigger.whileFalse(new InstantCommand(m_drivetrain::unlock));

  }

  public Command getAutonomousCommand() {
      return null;
  }


}
