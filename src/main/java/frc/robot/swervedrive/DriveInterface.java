package frc.robot.swervedrive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.Drivetrain;

import javax.sound.midi.Sequence;
import java.util.TooManyListenersException;
import java.util.function.Supplier;

public class DriveInterface {
    private boolean FeildOrinted = true;
    private String Auto_Path = "New Path";
    Drivetrain m_Drivetrain;
    DrivetrainCommand m_DriveKinamatics;
    DriveAuto m_DriveAuto;
    public DriveInterface(
            Supplier<Double> X, Supplier<Double> Y, Supplier<Double> Rotate,
            Supplier<Boolean> FeildOrinted, Supplier<Boolean> ToggleFeildOrinted,
            Supplier<Boolean> ZeroGryoBind){

        m_Drivetrain = new Drivetrain();
        m_DriveKinamatics = new DrivetrainCommand(
                m_Drivetrain,
                () -> Y.get() * Constants.MaxSpeed,
                () -> X.get() * Constants.MaxSpeed,
                () -> -Rotate.get(),
                () -> FeildOrinted.get()
        );

        m_Drivetrain.setDefaultCommand(m_DriveKinamatics);

        Trigger Left_Trigger = new Trigger(() -> ZeroGryoBind.get());
        Trigger FeildOrintedTigger = new Trigger(() -> ToggleFeildOrinted.get());
        Left_Trigger.whileTrue(new InstantCommand( () -> m_Drivetrain.Zero()));
        FeildOrintedTigger.whileTrue(new StartEndCommand(() ->m_Drivetrain.Lock_Motor() , () -> m_Drivetrain.Unlock()));
        GenerateAutoPath();
    }
    public Pose2d GetPos()
    {
        return m_Drivetrain.getPose();
    }


    public Command Drive(Pose2d NewPos)
    {
        Pose2d OldPos = GetPos();
        Transform2d transform = new Transform2d(OldPos, NewPos);
        double X, Y, R;
        X = transform.getX();
        Y = transform.getY();
        R = transform.getRotation().getRadians();
        double Time = 0;
        if (X > Y)
        {
            Time = Constants.Auto.MaxSpeed / X;
        } else if (X < Y) {
            Time = Constants.Auto.MaxSpeed / Y;
        }else{
            Time = Constants.Auto.MaxSpeed / X;
        }
        double TurnTime = R / Constants.MaxTurnSpeed;

        X = X * (Time /Constants.Auto.MaxSpeed);
        Y = Y * (Time /Constants.Auto.MaxSpeed);
        R = R * (TurnTime /Constants.Auto.TurningSpeed);

        System.out.println(X);
        System.out.println(Y);
        System.out.println(R);
        System.out.println("Hi\n");

        double finalX = X;
        double finalY = Y;
        double finalR = R;
        return new SequentialCommandGroup(
                new InstantCommand(() ->  m_DriveKinamatics.Drive( finalX, finalY, finalR, true)),
                new WaitCommand(TurnTime),
                new InstantCommand(() -> m_DriveKinamatics.Drive(finalX, finalY, 0, true)),
                new WaitCommand(Time),
                new InstantCommand(() -> m_DriveKinamatics.Drive(0,0,0,true))
        );
    }

    public Command GetAutoPath(){return AutoPath;}
    private Command AutoPath;
    public void GenerateAutoPath()
    {
        int NumOfPos;

        PathPlannerTrajectory State = PathPlanner.loadPath("New Path", 4, 3);
        NumOfPos = State.getMarkers().size();

        Pose2d[] Pose2dList = new Pose2d[NumOfPos];
        Command[] CommandList = new Command[NumOfPos];
//        System.out.println(NumOfPos);
        for (int i = 0; i < NumOfPos; i++) {
            Pose2dList[i] = new Pose2d(State.getMarkers().get(i).positionMeters, State.getMarkers().get(0).positionMeters.getAngle());
            CommandList[i] = Drive(Pose2dList[i]);
            System.out.println(State.getMarkers().get(i).names);
//            System.out.println(Pose2dList[i].getX());
//            System.out.println(Pose2dList[i].getY());
//            System.out.println(Pose2dList[i].getRotation().getDegrees());
//            System.out.println("\n");
        }


        AutoPath = new InstantCommand(() -> {
            for (int i = 0; i < NumOfPos; i++) {
                CommandList[i].initialize();
                CommandList[i].execute();
                while (!CommandList[i].isFinished()){}
            }
        });
    }
}

