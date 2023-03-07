package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup
{

  public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry, PoseEstimatorSubsystem poser)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            // drivebase::getPose,
            poser::getCurrentPose,
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase));
  }

}
