package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;


import javax.annotation.processing.RoundEnvironment;
import java.awt.geom.Line2D;
import java.math.RoundingMode;
import java.util.function.Supplier;
import java.util.logging.Logger;


public class DriveAuto {

    private final Drivetrain drivetrain;
    Supplier<SwerveDriveKinematics> Drive;
    Command[] List = new Command[100];

    java.util.List<PathPlannerTrajectory> TerjectoryList;
    String Path;
    int Size = 0;
    public DriveAuto(String Path, Supplier<SwerveDriveKinematics> swerveDriveKinematics, Drivetrain drivetrain)
    {
        this.Drive = swerveDriveKinematics;
        this.Path = Path;
        this.drivetrain = drivetrain;

        TerjectoryList = PathPlanner.loadPathGroup(Path, 4,3);
        this.Size = TerjectoryList.size();

        GenerateList();
    }

    public Command Auto_Command()
    {
        return new InstantCommand( () -> RunList());
    }
    private void RunList()
    {
        for (int i = 0; i < this.Size; i++) {

            List[i].initialize();
            List[i].execute();
        }
        drivetrain.stopModules();
    }
    private void GenerateList()
    {
        Pose2d Pos1, Pos2;
        for (int i = 0; i < Size; i++) {
            PathPlannerTrajectory State = TerjectoryList.get(i);
            if (i == 0)
            {
                Pos1 = new Pose2d(State.getInitialState().poseMeters.getX(), State.getInitialState().poseMeters.getY(),State.getInitialState().holonomicRotation);
                Pos2 = new Pose2d(State.getState(i).poseMeters.getX(), State.getEndState().poseMeters.getY(),State.getEndState().holonomicRotation);
                List[i] = GenerateCommand(Pos1, Pos2);
                i++; // This will incermaent it twice
            }else{
                Pos1 = new Pose2d(State.getState(i - 1).poseMeters.getX(), State.getInitialState().poseMeters.getY(),State.getInitialState().holonomicRotation);
                Pos2 = new Pose2d(State.getState(i).poseMeters.getX(), State.getEndState().poseMeters.getY(),State.getEndState().holonomicRotation);
                List[i] = GenerateCommand(Pos1, Pos2);
            }

        }
    }

    private Command GenerateCommand(Pose2d Current, Pose2d New)
    {
        double Distince = Current.getTranslation().getDistance(New.getTranslation());
        double Time = Distince / Constants.Auto.MaxSpeed; // How long it will take to get from one place to another

        double X, Y, R;

        Transform2d Transform = new Transform2d(Current, New);

        X = Transform.getX() / 15.7;


        Y = Transform.getY() / Constants.Auto.MaxSpeed;
        X = Transform.getX() / Constants.Auto.MaxSpeed;

        R = Current.getRotation().getRadians() - New.getRotation().getRadians();
        R = R / (Time/Constants.Auto.TurningSpeed);


        System.out.println("\n\n\n\n" +X + "\n" + Y + "\n");


        SwerveModuleState[] Target = Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(X, Y, R));
        SwerveModuleState[] Stop = Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setModuleStates(Target)),
                new InstantCommand(() -> new WaitCommand(Time)),
                new InstantCommand(() -> drivetrain.setModuleStates(Stop))
        );
    }
}

