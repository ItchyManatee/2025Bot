// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
    //private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    public final CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem = new CoralGroundIntakeSubsystem(); // #DJ032625 change to public

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_opController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
                () -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                        true),
                m_robotDrive));

        m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

        //m_climberSubsystem.setDefaultCommand(m_climberSubsystem.moveClimber(0));

        //m_coralGroundIntakeSubsystem.setDefaultCommand(m_coralGroundIntakeSubsystem.idleCommand());

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        m_opController.a().whileTrue(m_algaeSubsystem.runIntakeCommand());

        m_opController.b().whileTrue(m_algaeSubsystem.reverseIntakeCommand());

        //m_opController.leftTrigger().whileTrue(m_climberSubsystem.moveClimber(-0.5));

        //m_opController.rightTrigger().whileTrue(m_climberSubsystem.moveClimber(0.15));

        m_opController.leftBumper().whileTrue(m_coralGroundIntakeSubsystem.movePivot(-0.25));

        m_opController.rightBumper().whileTrue(m_coralGroundIntakeSubsystem.movePivot(0.65));

        m_opController.y().whileTrue(m_coralGroundIntakeSubsystem.moveIntake(0.5));

        m_opController.x().whileTrue(m_coralGroundIntakeSubsystem.moveIntake(-0.5));

        // Move to score position with POV 0
        m_opController
            .pov(90)
            .onTrue(
                m_coralGroundIntakeSubsystem.moveToScorePosition()
                    .withName("MoveToScorePosition"));
        
        // Move to catch position with POV 270
        m_opController
        .pov(270)
        .onTrue(
            m_coralGroundIntakeSubsystem.moveToCatchPosition()
                .withName("MoveToCatchPosition"));
                
        // Move to ground position with POV 90
        m_opController
            .pov(180)
            .onTrue(
                m_coralGroundIntakeSubsystem.moveToGroundPosition()
                    .withName("MoveToGroundPosition"));
        
        // Move to stow position with POV 180
        m_opController
            .pov(0)
            .onTrue(
                m_coralGroundIntakeSubsystem.moveToStowPosition()
                    .withName("MoveToStowPosition"));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1, 0)),
                // End 2 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

        // From WPILib
        var trajectoryOne =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(-1.1, 0.1), new Translation2d(-1.65, -0.1)),
            new Pose2d(-2.0, 0., Rotation2d.fromDegrees(0.20)),
            config);
        // var trajectoryTwo =
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        //     List.of(new Translation2d(4, 4), new Translation2d(6, 3)),
        //     new Pose2d(6, 0, Rotation2d.fromDegrees(0)),
        //     new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
        // var concatTraj = trajectoryOne.concatenate(trajectoryTwo);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                // exampleTrajectory,
                trajectoryOne,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
