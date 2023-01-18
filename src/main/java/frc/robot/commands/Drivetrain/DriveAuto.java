package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import javax.sound.sampled.Line;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;


public class DriveAuto {

    private final Drivetrain drivetrain;
    Supplier<SwerveDriveKinematics> Drive;
    SequentialCommandGroup[] List;
    String Path;
    public DriveAuto(String Path, Supplier<SwerveDriveKinematics> swerveDriveKinematics, Drivetrain drivetrain)
    {
        this.Drive = swerveDriveKinematics;
        this.Path = Path;
        this.drivetrain = drivetrain;

    }

    public Command Auto_Command()
    {
        return new InstantCommand( () -> RunList());
    }
    public void RunList()
    {

    }
    public void GenerateList()
    {
        java.util.List<PathPlannerTrajectory> TerjectoryList = PathPlanner.loadPathGroup(Path, 4,3);
        int Size = TerjectoryList.size();
        Pose2d Pos1, Pos2;
        for (int i = 0; i < Size; i++) {
            PathPlannerTrajectory State = TerjectoryList.get(i);

            Pos1 = new Pose2d(State.getInitialState().poseMeters.getX(), State.getInitialState().poseMeters.getY(),State.getInitialState().holonomicRotation);
            Pos2 = new Pose2d(State.getEndState().poseMeters.getX(), State.getEndState().poseMeters.getY(),State.getEndState().holonomicRotation);

            GenerateCommand(Pos1, Pos2);
        }
    }

    Command GenerateCommand(Pose2d Current, Pose2d New)
    {
        double Distince = Current.getTranslation().getDistance(New.getTranslation());
        double Time = Distince / Constants.Auto.MaxSpeed; // How long it will take to get from one place to another

        LineClass m_Lineclass = new LineClass(Current, New);

        double X, Y;
        X = m_Lineclass.HighestOutsidePoint().getX();
        Y = m_Lineclass.HighestOutsidePoint().getY();

        double Turn = 0;

        return new SequentialCommandGroup(
                new InstantCommand(() -> Drive.get().Drive(X, Y, Turn,true)),
                new InstantCommand(() -> new WaitCommand(Time))
        );
    }
}


class LineClass
{
    double Slope;
    double Yintercept;
    Pose2d First, Second;
    public LineClass(Pose2d First, Pose2d Second)
    {
        this.First = First;
        this.Second = Second;

        Slope = (First.getY() - Second.getY() ) / (First.getX() - Second.getX() );
        Yintercept = First.getY() - Slope * First.getX();
    }

    private double Yintercect(double Y)
    {
        double X = 0;

        return X;
    }
    private double Xintercect(double X)
    {
        double Y = 0;

        return Y;
    }


    public Translation2d HighestOutsidePoint()
    {
        double x = 0, y = 0;
        if (Second.getX() > 0) // Postive
        {

        }
        if (Second.getX() < 0) // Negitive
        {

        }

        return new Translation2d(x,y);
    }
    public double GetSlope()
    {
        return Slope;
    }

}

