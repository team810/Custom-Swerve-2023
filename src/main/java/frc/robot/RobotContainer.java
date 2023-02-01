package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.m_kinematics;

public class RobotContainer {
  private final PathPlannerTrajectory Path = PathPlanner.loadPath("New Path",4,3);
//  private final Joystick right = new Joystick(0);
//  private final Joystick left = new Joystick(1);
  private final XboxController xboxController = new XboxController(0);

  private final Drivetrain m_drivetrain = new Drivetrain();

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new DrivetrainCommand(
            m_drivetrain,
            () -> Math.pow(xboxController.getLeftY(), 3) * Constants.MaxSpeed, // Y
            () -> Math.pow(xboxController.getLeftX(), 3) * Constants.MaxSpeed, // X
            () -> Math.pow(-1 * xboxController.getRightX(), 3) * Constants.MaxTurnSpeed, // Z
            () -> true
    ));

    configureButtonBindings();
  }
  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {


      PathPlannerTrajectory path = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
              .5,
              .5)
              .setKinematics(m_kinematics);
      List<Pose2d> posList = new ArrayList<>();
      for (int i = 0; i < path.getStates().size(); i++) {
        posList.set(i, path.getState(i).poseMeters);
      }

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(posList,trajectoryConfig);

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(Constants.Auto.K_XController, 0, 0);
      PIDController yController = new PIDController(Constants.Auto.K_YController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              Constants.Auto.K_RController, 0, 0, new TrapezoidProfile.Constraints(4,3));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // 4. Construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory,
              m_drivetrain::getpos,
              m_kinematics,
              xController,
              yController,
              thetaController,
              m_drivetrain::setModuleStates,
              m_drivetrain);

      // 5. Add some init and wrap-up, and return everything
      return new SequentialCommandGroup(
              new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())),
              swerveControllerCommand,
              new InstantCommand(m_drivetrain::stopModules)
      );

  }


}
