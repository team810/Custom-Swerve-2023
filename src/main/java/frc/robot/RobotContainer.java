package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swervedrive.DriveInterface;

public class RobotContainer {
  private final Joystick right = new Joystick(0);
  private final Joystick left = new Joystick(1);
  private DriveInterface m_Drive;

  public RobotContainer() {
    m_Drive = new DriveInterface(
            () -> right.getX(),
            () -> right.getY(),
            () -> left.getTwist(),
            () -> true,
            () -> true,
            () -> left.getTriggerPressed()
    );

    configureButtonBindings();
  }
  private void configureButtonBindings() {


  }

  public Command getAutonomousCommand() {

//   return m_DriveAuto.Auto_Command();
    return m_Drive.GetAutoPath();

  }
}
