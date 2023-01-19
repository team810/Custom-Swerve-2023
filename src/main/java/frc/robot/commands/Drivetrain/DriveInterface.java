package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.function.Supplier;

public class DriveInterface {
    private boolean FeildOrinted = true;
    private String Auto_Path = "New Path";
    Drivetrain m_Drivetrain;
    SwerveDriveKinematics m_DriveKinamatics;
    DriveAuto m_DriveAuto;
    public DriveInterface(Supplier<Joystick> Left, Supplier<Joystick> Right)
    {
        m_Drivetrain = new Drivetrain();
        m_DriveKinamatics = new SwerveDriveKinematics(
                m_Drivetrain,
                () -> Right.get().getX(),
                () -> Right.get().getY(),
                () -> Left.get().getTwist(),
                () -> FeildOrinted
        );
        m_DriveAuto = new DriveAuto(
                Auto_Path, // This is the name of the path that the robot is going to follow
                () -> m_DriveKinamatics,
                m_Drivetrain
        );

    }


}
