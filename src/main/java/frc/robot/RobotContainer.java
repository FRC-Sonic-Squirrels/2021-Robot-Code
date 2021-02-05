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
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.pwmConstants;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.blinkinSubsystem;
import frc.robot.commands.*;

public class RobotContainer {

  // Position of Power Port, needs to be set in each auton routine
  public Translation2d powerPortLocation = new Translation2d(feet2Meters(10), 0);

  // Subsystems
  // NOTE: blinkin needs to be first and public static to be accessed by other subsystems
  public final static blinkinSubsystem m_blinkin = new blinkinSubsystem(pwmConstants.blinkin);
  // All other subsystems should be private
  public final driveSubsystem m_drive = new driveSubsystem();
  // public so that it can get the right instance.
  public static final limelight m_limelight = new limelight("limelight-one");
  private final turretSubsystem m_turret = new turretSubsystem();
  public final shooterSubsystem m_shooter = new shooterSubsystem();
  public static final indexerSubsystem m_indexer = new indexerSubsystem();
  //private final elevatorSubsystem m_elevator = new elevatorSubsystem();
  //private final controlPanelSubsystem m_controlPanelMotors = new controlPanelSubsystem();
  private final intakeSubsystem m_intake = new intakeSubsystem();
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
    final JoystickButton driverXButton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton driverYButton = new JoystickButton(m_driveController, Button.kY.value);
    final JoystickButton driverStartButton = new JoystickButton(m_driveController, Button.kStart.value);
    final JoystickButton driverBackButton = new JoystickButton(m_driveController, Button.kBack.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driveController, Button.kBumperLeft.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driveController, Button.kBumperRight.value);
    
    // Operator Controller Buttons
    final JoystickButton opAButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton opBButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton opXButton = new JoystickButton(m_operatorController, Button.kX.value);
    final JoystickButton opYButton = new JoystickButton(m_operatorController, Button.kY.value);
    final JoystickButton opStartButton = new JoystickButton(m_operatorController, Button.kStart.value);
    final JoystickButton opBackButton = new JoystickButton(m_operatorController, Button.kBack.value);
    final JoystickButton opLeftBumper = new JoystickButton(m_operatorController, Button.kBumperLeft.value);
    final JoystickButton opRightBumper = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final POVButton opDPadUp = new POVButton(m_operatorController, 0);
    final POVButton opDPadDown = new POVButton(m_operatorController, 180);
    
    // Driver Controls
      // Y Button to deploy the elevator
      // X Button to retract the elevator
      // A Button toggle Forza mode
      // B Button toggle square driver inputs
      // Left Trigger - climber down (raise robot)
      // Right Trigger - climber up (lower robot)
      // Right Bumper - invert drive controls
      // Left Bumper - Slow down robot by 1/2
      //driverYButton.whenPressed(new InstantCommand(() -> m_turret.setAngleDegrees(0), m_turret).andThen(() -> m_elevator.deployElevator()));
      //driverXButton.whenPressed(() -> m_elevator.retractElevator());
      driverRightBumper.whenPressed(new driveInvertCommand(m_drive));
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
      opAButton.whileHeld(new intakeDeployCommand(m_intake));
      opBButton.whenPressed(new indexerStageForShootingCommand(m_indexer));
      opXButton.whenPressed(new indexerRestageCommand(m_indexer));
      opYButton.whileHeld(new indexerReverseEjectCommand(m_indexer));
      opRightBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight));
      //opLeftBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight));
      opLeftBumper.whileHeld(new shooterUnderGoal(m_indexer, m_turret, m_shooter));
      opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
      opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));
      opBackButton.whenPressed(new shooterSpoolCommand(m_shooter));
  }
  
  /**
   * Do nothing during auton
   * 
   * @return Auton do nothing command
   */
  public Command noAutonomous() {
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }

  public Command straightOn3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);
    
    return moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter)).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
  }

  public Command straightOn3BallForward() {
    RamseteCommand moveForward1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(0.25, 0)), new Pose2d(0.5, 0, new Rotation2d(0)), false, 2.5, 0.75);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.000000, 0, 0.00018, 250), m_shooter).
    andThen(moveForward1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(2800), m_shooter))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, false));
  }

  public Command straightOn6BallRendezvous() {//Gets the 3 balls on rendezvous point towards center of field
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-1, 0, new Rotation2d(0.4)), List.of(new Translation2d(-1, 0)), new Pose2d(-2, 0, new Rotation2d(0)), true, 2.5, 0.5); 
    RamseteCommand moveForward3 = createTrajectoryCommand(new Pose2d(-2, 0, new Rotation2d(0)), List.of(new Translation2d(-1, 0)), new Pose2d(0, 0, new Rotation2d(0)), false, 2.5, 1);
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).alongWith(new InstantCommand(() -> m_indexer.setBallCount(3), m_indexer)).
    andThen((new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true))).
    andThen(moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter))).
    andThen(new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually()).
    andThen(moveForward3.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(3.0), m_turret), new indexerStageForShootingCommand(m_indexer))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true)));
  }

  // same as straightOn3Ball() but with sing SequentialCommandGroup
  public Command straightOn3Ball_reorg() {
   
    // power port is directly in front of robot, center limelight over initiation line
    powerPortLocation = new Translation2d(feet2Meters(10), 0);

    Command ac = new SequentialCommandGroup(
      // Do these setup things in parallel
      new ParallelCommandGroup(
        new InstantCommand(() -> m_indexer.setBallCount(3), m_indexer),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter),  // aprox rpm for 13'
          new InstantCommand(() -> m_shooter.deployHood(), m_shooter)    // deploy hood, set ll pipeline
        ),
        createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75)
      ),
      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true, true)
    );

    return ac;
  }


  public Command rightSide3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.1, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);

    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
  }

  public Command rightSide4Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.54, 0, new Rotation2d(0)), true, 2.5, 1);
    RamseteCommand moveForward2 = createTrajectoryCommand(new Pose2d(-2.54, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(0, 0, new Rotation2d(0)), false, 3.5, 1.5);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually(), new InstantCommand(() -> m_indexer.runIntake()))).
    andThen(moveForward2.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret)).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true)));
  }

  public Command rightSide5Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.54, 0, new Rotation2d(0)), true, 2, 1);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-2.54, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-3.5, 0, new Rotation2d(0)), true, 2.5, 1);
    RamseteCommand moveForward3 = createTrajectoryCommand(new Pose2d(-2.85, 0, new Rotation2d(0)), List.of(new Translation2d(-2, 0)), new Pose2d(-1.5, 0, new Rotation2d(0)), false, 2.5, 1);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitCommand(2).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward3.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
  }

  public Command rightSide6Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.49, 0, new Rotation2d(0)), true, 3, 2);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-2.49, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-3.35, 0, new Rotation2d(0)), true, 3, 1.5);
    RamseteCommand moveBack3 = createTrajectoryCommand(new Pose2d(-3.35, 0, new Rotation2d(0)), List.of(new Translation2d(-3.6, 0)), new Pose2d(-4.35, 0, new Rotation2d(0)), true, 3, 1.5);
    RamseteCommand moveForward4 = createTrajectoryCommand(new Pose2d(-4.35, 0, new Rotation2d(0)), List.of(new Translation2d(-4, 0)), new Pose2d(-1.5, 0, new Rotation2d(0)), false, 3.75, 2.75);

    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).alongWith(new InstantCommand(() -> m_indexer.setBallCount(3))).
    andThen(new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret)).
    andThen(new WaitUntilCommand(() -> m_indexer.getBallCount() == 0).deadlineWith(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true))).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitCommand(0.5).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack3.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually()).
    andThen(new WaitCommand(0.75).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward4.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer)))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
  }

  public Command rightSide6Ball_reorg() {

    // power port is left of robot, 
    // 1. front of frame over initiation line
    // 2. robot lined up on row of balls
    powerPortLocation = new Translation2d(feet2Meters(10.5), inches2Meters(66.91));

    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-1.75, 0, new Rotation2d(0)), true, 3.0, 1.8); 
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-1.75, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-4.35, 0, new Rotation2d(0)), true, 0.80, 0.5);
    RamseteCommand moveForward = createTrajectoryCommand(new Pose2d(-4.35, 0, new Rotation2d(0)), List.of(new Translation2d(-4, 0)), new Pose2d(-2, 0, new Rotation2d(0)), false, 3.0, 1.8);

    Command ac = new SequentialCommandGroup(
      // Do these setup things in parallel
      new ParallelCommandGroup(
        new InstantCommand(() -> m_indexer.setBallCount(3), m_indexer),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_shooter.setShooterRPM(3650), m_shooter),  // aprox rpm for 17'
          new InstantCommand(() -> m_shooter.deployHood(), m_shooter)    // deploy hood, set ll pipeline
        ),
        new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), // look left
        moveBack1),   // Move left

      // shoot until all the balls are gone
      new ParallelRaceGroup(
        new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true, true),
        new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)
      ),

      // Move back get 3 more balls
      
      new ParallelRaceGroup(
        new indexerDefaultCommand(m_indexer).perpetually(), 
        new intakeDeployCommand(m_intake),
        new SequentialCommandGroup(
          moveBack2                    // move back slow
          //new WaitCommand(0.1)          // time to finish sucking in last ball
        )
      ),

      // Move forward prepare to shoot
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          // no need to turn turret, should still be on target from last time
          new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter),  // aprox rpm for 17'
          new InstantCommand(() -> m_shooter.deployHood(), m_shooter)    // deploy hood, set ll pipeline
        ),
        new indexerStageForShootingCommand(m_indexer),
        moveForward
      ),

      // shoot, finish when all the balls are gone
      new ParallelRaceGroup(
        new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true, true),
        new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)
      )

    );

    return  ac;
  }


  public Command rightSide6BallTest() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-1.75, 0, new Rotation2d(0)), true, 2.5, 1.5);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-1.75, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-4.35, 0, new Rotation2d(0)), true, 0.75, 0.5);
    RamseteCommand moveForward3 = createTrajectoryCommand(new Pose2d(-4.35, 0, new Rotation2d(0)), List.of(new Translation2d(-4, 0)), new Pose2d(-2, 0, new Rotation2d(0)), false, 2.5, 1.5);

    // To try and save time, this version continues through the 3 balls slowly rather than stopping at each one
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    alongWith(new InstantCommand(() -> m_indexer.setBallCount(3))).
    andThen(new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_limelight.setLEDMode(0))).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitUntilCommand(() -> m_indexer.getBallCount() == 0).deadlineWith(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true))).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward3.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer), new InstantCommand(() -> m_limelight.setLEDMode(0)))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true, true));
  }

  public Command middle3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.1, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(3), m_turret))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
  }

  public Command middle4BallTest() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.75, -0.25)), new Pose2d(-2.54, -0.5, new Rotation2d(0)), true, 3.5, 1.5);
    RamseteCommand moveForward2 = createTrajectoryCommand(new Pose2d(-2.54, -0.5, new Rotation2d(0)), List.of(new Translation2d(-0.75, -0.25)), new Pose2d(0, 0, new Rotation2d(0)), true, 3.5, 1.5);
    
    return
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward2.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(3)))).
    andThen(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_limelight, true));
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
