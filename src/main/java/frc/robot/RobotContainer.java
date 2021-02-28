/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.driveConstants.kDriveKinematics;

import java.util.List;

import com.fearxzombie.limelight;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.indexerDefaultCommand;
import frc.robot.commands.indexerStageForShootingCommand;
import frc.robot.commands.shooterAutoCommand;
import frc.robot.commands.turretDefaultCommand;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class RobotContainer {

  // Position of Power Port, needs to be set in each auton routine
  public Translation2d powerPortLocation = new Translation2d(feet2Meters(10), 0);

  // Subsystems
  // All other subsystems should be private
  public final driveSubsystem m_drive = new driveSubsystem();

  // public so that it can get the right instance.
  public static final limelight m_limelight = new limelight("limelight-one");
  private final turretSubsystem m_turret = new turretSubsystem();
  public final shooterSubsystem m_shooter = new shooterSubsystem();
  private static final indexerSubsystem m_indexer = new indexerSubsystem();
  private final intakeSubsystem m_intake = new intakeSubsystem(m_drive);
  private final hoodSubsystem m_hood = new hoodSubsystem();
  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public static boolean limelightOnTarget = false;


  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(new driveCommand(m_drive));
    //m_elevator.setDefaultCommand(new elevatorWinchCommand(m_elevator));
    m_indexer.setDefaultCommand(new indexerDefaultCommand(m_indexer));
    m_turret.setDefaultCommand(new turretDefaultCommand(m_turret));
  }

  private void configureButtonBindings() {
    
    // Driver Controller Buttons
    final JoystickButton driverAButton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton driverBButton = new JoystickButton(m_driveController, Button.kB.value);
    //final JoystickButton driverXButton = new JoystickButton(m_driveController, Button.kX.value);
    //final JoystickButton driverYButton = new JoystickButton(m_driveController, Button.kY.value);
    //final JoystickButton driverStartButton = new JoystickButton(m_driveController, Button.kStart.value);
    //final JoystickButton driverBackButton = new JoystickButton(m_driveController, Button.kBack.value);
    //final JoystickButton driverLeftBumper = new JoystickButton(m_driveController, Button.kBumperLeft.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driveController, Button.kBumperRight.value);
    
    // Operator Controller Buttons
    final JoystickButton opAButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton opBButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton opXButton = new JoystickButton(m_operatorController, Button.kX.value);
    final JoystickButton opYButton = new JoystickButton(m_operatorController, Button.kY.value);
    //final JoystickButton opStartButton = new JoystickButton(m_operatorController, Button.kStart.value);
    final JoystickButton opBackButton = new JoystickButton(m_operatorController, Button.kBack.value);
    final JoystickButton opLeftBumper = new JoystickButton(m_operatorController, Button.kBumperLeft.value);
    final JoystickButton opRightBumper = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final POVButton opDPadUp = new POVButton(m_operatorController, 0);
    final POVButton opDPadDown = new POVButton(m_operatorController, 180);
    
    // Driver Controls
      // A Button toggle Forza mode
      // B Button toggle square driver inputs
      // Left Trigger - reverse throttle (Forza mode)
      // Right Trigger - forward throttle (Forza mode)
      // Right Bumper - invert drive controls
      // Left Bumper - turbo boost, FULL SPEED
      driverRightBumper.whenPressed(new InstantCommand(() -> m_drive.toggleDriveInverted()));
      driverAButton.whenPressed(new InstantCommand(() -> m_drive.toggleForzaMode()));
      driverBButton.whenPressed(new InstantCommand(() -> m_drive.toggleSquaredInputs()));

    // Operator Controls
      // Left Joystick - manual turret control
      // Left Trigger - manually move the indexer backwards
      // Right Trigger - manually move the indexer forwards
      // A Button - hold to deploy intake
      // B Button - stage balls for shooting
      // X Button - restage balls
      // Y Button - hold to eject balls out the back of the indexer
      // Right Bumper - shoot
      // Left Bumper - shoot
      // D Pad Up - manually increase ball count
      // D Pad Down - manually decrease ball count
      // Start Button - zero the turret
      // Back Button - spool up the shooter
      //opBButton.whenPressed(new indexerStageForShootingCommand(m_indexer));
      //opXButton.whenPressed(new indexerRestageCommand(m_indexer));
      //opYButton.whileHeld(new indexerReverseEjectCommand(m_indexer));

      opRightBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight));

      //opLeftBumper.whileHeld(new shooterUnderGoal(m_indexer, m_turret, m_shooter));
      //opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
      //opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));

      opRightBumper.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0)));

      opAButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3700))); // 5 feet
      opBButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(5000))); // 10 feet
      opXButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(5400))); // 15 feet
      opYButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(6000))); // 20 feet

  // alternative shooter speed debug
  //    opAButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(1000)));
  //    opBButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(-1000)));
  //    opXButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(100)));
  //    opYButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(-100)));

 }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoName) {

    if (autoName == "donothing") {
      return getNoAutonomousCommand();
    }
    else if (autoName == "figure8") {
      // return getAutonomousFigure8Command();
    }
    else if (autoName == "barrel") {
      // return getAutonomousBarrelCommand();
    }
    else if (autoName == "slalom") {
      //return getAutonomousSlalomCommand();
    }
    else if (autoName == "bounce") {
      //return getAutonomousBounceCommand();
    }

    // return do nothing if we don't recognize the choice
    System.out.println("Warning: No autonomous command specified.");
    return getNoAutonomousCommand();
  }

  /**
   * Do nothing during auton
   * 
   * @return Auton do nothing command
   */
  public Command getNoAutonomousCommand() {
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * createTrajectoryCommand - given a start pose, some intermediate points, and a finish pose, create
   *     a Ramsete Command to execute the path follow.
   * 
   * @param startPose
   * @param translationList
   * @param endPose
   * @param isReversed
   * @param maxSpeedMetersPerSecond
   * @param maxAccelerationMetersPerSecondSquared
   * @return Ramsete Path Follow Command, intake side of robot is isReversed = true and negative values
   */
  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose, boolean isReversed, double maxSpeedMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(isReversed);

    var initialTime = System.nanoTime();

    // trajectory to follow. All units in meters.
    var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(trajectory, 
            m_drive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            m_drive.getFeedforward(),
            kDriveKinematics,
            m_drive::getWheelSpeeds,
            m_drive.getLeftPidController(),
            m_drive.getRightPidController(),
            m_drive::tankDriveVolts,
            m_drive);

    var dt = (System.nanoTime() - initialTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }
  

  // TODO: this should be in com/team2930/utils/units.java or a new Units.java under Utils
  public double inches2Meters(double i) {
    return i * 0.0254;
  }

  public double feet2Meters(double feet) {
    return (feet * 0.3048);
  }

}
