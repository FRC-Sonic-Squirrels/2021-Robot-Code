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

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import com.fearxzombie.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.indexerDefaultCommand;
import frc.robot.commands.indexerReverseCommand;
import frc.robot.commands.intakeDeploy;
import frc.robot.commands.shooterAutoCommand;
import frc.robot.commands.shooterUnderGoal;
import frc.robot.commands.turretDefaultCommand;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class RobotContainer {

  SendableChooser<Command> chooser = new SendableChooser<>();

  // Subsystems
  // All other subsystems should be private
  public final driveSubsystem m_drive = new driveSubsystem();

  // public so that it can get the right instance.
  public static final limelight m_limelight = new limelight("limelight-one");
  //public static final limelight m_limelightPowerCell = new limelight("limelight");
  private final turretSubsystem m_turret = new turretSubsystem();
  public final shooterSubsystem m_shooter = new shooterSubsystem();
  private static final indexerSubsystem m_indexer = new indexerSubsystem();
  public final intakeSubsystem m_intake = new intakeSubsystem(m_drive);
  private final hoodSubsystem m_hood = new hoodSubsystem();
  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public static boolean limelightOnTarget = false;
  public static String selectedPath = "";
  public static Supplier<Object> i = () -> "";

  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(new driveCommand(m_drive));
    m_indexer.setDefaultCommand(new indexerDefaultCommand(m_indexer));
    //m_turret.setDefaultCommand(new turretDefaultCommand(m_turret));

    chooser.addOption("Move Back 3 Shoot", moveThenShoot(3.0));
    //chooser.addOption("Right Side 6", rightSide6Ball());
    chooser.setDefaultOption("Right Side 6", rightSide6Ball());
    chooser.addOption("Move Forward 2 Shoot", moveThenShoot(-2.0));
    chooser.addOption("Go Forward 1m", autonCalibrationForward(1.0));
    chooser.addOption("Go Forward 2m", autonCalibrationForward(2.0));
    chooser.addOption("Go Back 1m", autonCalibrationForward(-1.0));
    //chooser.addOption("Go Forward 3", autonCalibrationForward(3.0));
    //chooser.addOption("Shooter Test", getAutonomousToTarget());
    //chooser.addOption("Curve Left", autonCalibrationCurve(1.0, 1.0));
    //chooser.addOption("Curve Right", autonCalibrationCurve(1.0, -1.0));
    chooser.addOption("Do Nothing", getNoAutonomousCommand());
    //chooser.setDefaultOption("Do Nothing", getNoAutonomousCommand());
    SmartDashboard.putData("Auto mode", chooser);

  }

  private void configureButtonBindings() {

    // Driver Controller Buttons
    final JoystickButton driverAButton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton driverBButton = new JoystickButton(m_driveController, Button.kB.value);
    final JoystickButton driverXButton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton driverYButton = new JoystickButton(m_driveController, Button.kY.value);
    final JoystickButton driverStartButton = new JoystickButton(m_driveController, Button.kStart.value);
    final JoystickButton driverBackButton = new JoystickButton(m_driveController, Button.kBack.value);
    // driver left bumper is hardcoded to be turbo boost
    final JoystickButton driverLeftBumper = new JoystickButton(m_driveController, Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driveController, Button.kRightBumper.value);

    // Operator Controller Buttons
    final JoystickButton opAButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton opBButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton opXButton = new JoystickButton(m_operatorController, Button.kX.value);
    final JoystickButton opYButton = new JoystickButton(m_operatorController, Button.kY.value);
    final JoystickButton opStartButton = new JoystickButton(m_operatorController, Button.kStart.value);
    final JoystickButton opBackButton = new JoystickButton(m_operatorController, Button.kBack.value);
    final JoystickButton opLeftBumper = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    final JoystickButton opRightBumper = new JoystickButton(m_operatorController, Button.kRightBumper.value);
    final POVButton opDPadUp = new POVButton(m_operatorController, 0);
    final POVButton opDPadDown = new POVButton(m_operatorController, 180);

    // Driver Controls
    // A Button  - 
    // B Button  - invert drive controls
    // X Button  - toggle curvature drive mode
    // Y Button  - 
    // Left Trigger  - reverse throttle (Forza mode)  
    // Right Trigger - forward throttle (Forza mode)
    // Left Bumper   - turbo boost, FULL SPEED 
    // Right Bumper  - 
    // driverRightBumper.whenPressed(new InstantCommand(() -> m_drive.toggleDriveInverted()));
    driverLeftBumper.whenPressed(new InstantCommand(() -> m_drive.toggleDriveInverted()));
    //driverXButton.whenPressed(new InstantCommand(() -> m_drive.toggleCurvatureMode()));

    // driverAButton.whenPressed(new InstantCommand(() -> m_drive.toggleForzaMode()));
    // driverBButton.whenPressed(new InstantCommand(() -> m_drive.toggleSquaredInputs()));
    // driverXButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));

    // driverAButton.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake));
    // driverXButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter));
    // driverBackButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0), m_shooter));
    // driverBButton.whenPressed(intakeReleaseCommand());

    driverXButton.whenPressed(shooterStopCommand());
    driverYButton.whenPressed(new InstantCommand(() -> m_indexer.stopIndexer()));
    driverBButton.whenPressed(new InstantCommand(() -> m_turret.turretHome()));
    driverAButton.toggleWhenPressed(new intakeDeploy(m_intake, m_indexer, 200));

    driverRightBumper.whileHeld(new shooterUnderGoal(m_indexer, m_turret, m_shooter, m_hood));

    // Operator Controls
    // Left Joystick - manual turret control
    // Left Trigger - manually move the indexer backwards
    // Right Trigger - manually move the indexer forwards
    // A Button - deploy intake while Held
    // B Button - stop indexer
    // X Button - 
    // Y Button - enable normal indexer ball intake mode 
    // Right Bumper - full auto shoot 
    // Left Bumper - shoot from under goal 
    // D Pad Up - manually increase ball count
    // D Pad Down - manually decrease ball count
    // Start Button - reset Odometry
    // Back Button - spool up the shooter

    //opBButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));
    //opBButton.whenPressed(new indexerStopCommand(m_indexer)); 
    //opXButton.whileHeld(new indexerReverseCommand(m_indexer));
    //opYButton.whenPressed(new InstantCommand(() -> m_indexer.setIntakeMode(), m_indexer));
    opXButton.whenPressed(shooterStopCommand());
    opYButton.whenPressed(new InstantCommand(() -> m_indexer.stopIndexer()));
    opBButton.whenPressed(new InstantCommand(() -> m_turret.turretHome()));
    opAButton.whileHeld(new intakeDeploy(m_intake, m_indexer, 200));

    //opLeftBumper.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter));
    opBackButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0), m_shooter));

    // left to shoot from under goal, right bumper for fully autonomous shooting
    opLeftBumper.whileHeld(new shooterUnderGoal(m_indexer, m_turret, m_shooter, m_hood));
    opRightBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake));

    opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
    opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));

    // Start button zeros robot pose and heading. Zeros encoders and gyro heading.
    opStartButton.whenPressed(() -> m_drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))), m_drive);

    // Shooter debug
    //opRightBumper.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0)));
    //opAButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3700))); // 5 feet
    //opBButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(5000))); // 10 feet
    //opXButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(5400))); // 15 feet
    //opYButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(6000))); // 20 feet

    // alternative shooter speed debug
    // opAButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(1000)));
    // opBButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(-1000)));
    // opXButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(100)));
    // opYButton.whenPressed(new InstantCommand(() -> m_shooter.addToShooterRPM(-100)));

 }
  
  
 public Command shooterStopCommand() {
  return new SequentialCommandGroup(
    new InstantCommand(() -> m_hood.retractHood()), 
    new InstantCommand(() -> m_shooter.setShooterRPM(0))
  );
 }


 public Command intakeStartCommand() {
  return new SequentialCommandGroup(
    new InstantCommand(() -> m_intake.deployIntake()), 
    new WaitCommand(0.7), 
    new InstantCommand(() -> m_indexer.setIntakeMode()),
    new InstantCommand(() -> m_intake.setDynamicSpeed(true))
  );
 }

 public Command intakeReleaseCommand() {
  return new SequentialCommandGroup(
    new InstantCommand(() -> m_intake.releaseIntake()), 
    new InstantCommand(() -> m_intake.retractIntake()), 
    //new WaitCommand(0.3), 
    new InstantCommand(() -> m_intake.setIntakePercentOutput(0.0)),
    new InstantCommand(() -> m_intake.deployIntake()), 
    //new InstantCommand(() -> m_intake.setIntakePercentOutput(-0.2)),
    new WaitCommand(0.2),
    new InstantCommand(() -> m_indexer.setIntakeMode()),
    new InstantCommand(() -> m_intake.setDynamicSpeed(true))
    );
 }

 public Command intakeRetractCommand() {
  return new SequentialCommandGroup(
    new InstantCommand(() -> m_intake.setDynamicSpeed(false)),
    new InstantCommand(() -> m_intake.setIntakePercentOutput(0.0)),
    new InstantCommand(() -> m_intake.retractIntake())
    );
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
   * Shoot 3, back up collect 3, move forward, and shoot 3
   * 
   * @return Auto command
   */
  public Command rightSide6Ball() {

    // power port is left of robot, 
    // 1. front of frame over initiation line
    // 2. robot lined up on row of balls
    //powerPortLocation = new Translation2d(feet2Meters(10.5), inches2Meters(66.91));

    Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d back1Pos = new Pose2d(1.75, 0, new Rotation2d(0));
    Pose2d back2Pos = new Pose2d(4.35, 0, new Rotation2d(0));
    Pose2d finalPos = new Pose2d(2, -0.5, new Rotation2d(0));

    Command moveBack1 = createTrajectoryCommand(startPos, List.of(), back1Pos, false, 3.0, 1.8); 
    Command moveBack2 = createTrajectoryCommand(back1Pos, List.of(), back2Pos, false, 1.0, 1.0);
    Command moveForward = createTrajectoryCommand(back2Pos, List.of(), finalPos, true, 3.0, 1.8);

    double goalDistanceFeet = 17.0;

    Command ac = new SequentialCommandGroup(
      // Do these setup things in parallel
      new ParallelCommandGroup(
        new InstantCommand(() -> m_indexer.setBallCount(3), m_indexer),
        new InstantCommand(() -> m_shooter.setShooterRPMforDistanceFeet(goalDistanceFeet), m_shooter),
        new InstantCommand(() -> m_hood.setPositionForDistanceFeet(goalDistanceFeet), m_hood),
        new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), // look left
        moveBack1
        ),

      // shoot until all the balls are gone
      new ParallelRaceGroup(
        new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(5),
        new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)
      ),

      // Move back get 3 more balls
      new ParallelRaceGroup(
        new intakeDeploy(m_intake, m_indexer, 200),
        moveBack2                    // move back slow
        //new WaitCommand(0.1)       // time to finish sucking in last ball
      ),


      new InstantCommand(() -> m_shooter.setShooterRPMforDistanceFeet(goalDistanceFeet), m_shooter),
      new InstantCommand(() -> m_hood.setPositionForDistanceFeet(goalDistanceFeet), m_hood),

      // Move forward prepare to shoot
      new ParallelCommandGroup(
          moveForward,
          new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(7)
      )

      // shoot, finish when all the balls are gone
  //    new ParallelRaceGroup(
  //      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(7),
  //      new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)
  //    )

    );

    return  ac;
  }

 /**
   * Back up designated distance in feet and shoot. Negative distance moves "backwards" towards goal.
   * 
   * @return Auto command
   */
  public Command moveThenShoot(double moveDistanceFeet) {

    // 1. power port is directly in front of robot, in sight of limelight
    // 2. front of frame over initiation line

    Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d pos1 = new Pose2d(Units.feetToMeters(moveDistanceFeet), 0, new Rotation2d(0));

    Command move1 = createTrajectoryCommand(startPos, List.of(), pos1, (moveDistanceFeet < 0.0), 3.0, 1.8); 

    double goalDistanceFeet = 10.0 - moveDistanceFeet;

    Command ac = new SequentialCommandGroup(
      // Do these setup things in parallel
      new ParallelCommandGroup(
        new InstantCommand(() -> m_indexer.setBallCount(3), m_indexer),
        new InstantCommand(() -> m_shooter.setShooterRPMforDistanceFeet(goalDistanceFeet), m_shooter),
        new InstantCommand(() -> m_hood.setPositionForDistanceFeet(goalDistanceFeet), m_hood),
        new InstantCommand(() -> m_turret.setAngleDegrees(0), m_turret),
        move1
        ),

      // shoot until all the balls are gone
      new ParallelRaceGroup(
        new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(10),
        new WaitUntilCommand(() -> m_indexer.getBallCount() == 0)
      ),

      // retract the hood and stop the flywheel
      shooterStopCommand()

    );

    return  ac;
  }


 /**
   * Forward Autonomous Command
   * 
   * Return an autonomous command that drives straight for a given distance
   * in meters.
   * 
   * @param distanceInMeters
   * @return Autonomous Command
   */
  public Command autonCalibrationForward(double distanceInMeters) {

    Pose2d startPose = new Pose2d(0.0, 0.0, new Rotation2d(0));

    Command ramseteCommand = createTrajectoryCommand(
        startPose, 
        List.of(),
        new Pose2d(distanceInMeters, 0.0, new Rotation2d(0)),
        (distanceInMeters < 0.0), 1.5, 0.75);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

 /**
   * Curve Autonomous Command
   *  
   * Return an autonomous command that drives forward and to the left/right for a given distances
   * in meters.
   * 
   * @param forwardInMeters
   * @param leftInMeters
   * @return Autonomous Command
   */
  public Command autonCalibrationCurve(double forwardInMeters, double leftInMeters) {

    double rotation = Math.PI/2;

    if (leftInMeters == 0) {
        rotation = 0;
    }
    else if (leftInMeters < 0)  {
      // turning tp the right
      rotation = -1.0 * Math.PI / 2.0;
    }

    Pose2d startPose = new Pose2d(0.0, 0.0, new Rotation2d(0));

    Command ramseteCommand = createTrajectoryCommand(
        startPose, 
        List.of(),
        new Pose2d(forwardInMeters, leftInMeters, new Rotation2d(rotation)),
        false, 1.5, 0.75);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousToTarget - Generate Auton Command used in Autonomous Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousToTarget(){
    //Start of Shooting with Red A
    Pose2d startPose = new Pose2d(inches2Meters(320), inches2Meters(95), new Rotation2d(0));

    Command ramseteCommand = createTrajectoryCommand(
      startPose,
      List.of(),
      new Pose2d(inches2Meters(90), inches2Meters(160), new Rotation2d(0)), true, 1.5, 0.5
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.setDynamicSpeed(false)),
      ramseteCommand, 
      new InstantCommand(() -> m_drive.tankDriveVolts(0, 0)),
      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(15),
      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake).withTimeout(5),
      new InstantCommand(() -> m_indexer.stopIndexer())
    );

  }

  /**
   * Figure 8 Autonomous Command
   * 
   * @return Command object
   */
  public Command getAutonomousFigure8Command() {
 
    // This assumes a start pose of (0,0) angle 0 (where ever the robot starts at)

    // distances are in Meters
    List<Translation2d> figure_eight = List.of(
      new Translation2d( 0.5, -0.5),
      new Translation2d( 1.0, -1.0),
      new Translation2d( 1.5, -0.5),
      new Translation2d( 1.0,  0.0),
      new Translation2d( 0.5, -0.5),
      new Translation2d( 0.0, -1.0),
      new Translation2d(-0.5, -0.5));

    // Start of a Figure 8
    Command ramseteCommand = createTrajectoryCommand(
        new Pose2d(0, 0, new Rotation2d(0)),
        figure_eight,
        new Pose2d(0.0, 0.0, new Rotation2d(0)), false, 1.0, 0.25);
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
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
  public Command createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose, boolean isReversed, double maxSpeedMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
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

    long initialTime = System.nanoTime();

    // trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    Command ramseteCommand =
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

    double dt = (System.nanoTime() - initialTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
        ramseteCommand);
  }
  
  /**
   * loadPathWeaverTrajectoryCommand - load a PathWeaver generated JSON file with
   * a trajectory and convert that to a Ramsete Command.
   * 
   * PathWeaver places JSON files in src/main/deploy/paths which will
   * automatically be placed on the roboRIO file system in
   * /home/lvuser/deploy/paths and can be accessed using getDeployDirectory.
   * 
   * Filename should be a string in the form of "paths/RobotPath1.json"
   * 
   * @param filename
   * @return Command
   */
  public Command loadPathWeaverTrajectoryCommand(String filename, boolean resetOdometry) {

    long initialTime = System.nanoTime();
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
      System.out.println("Unable to read from file " + filename );
      return new InstantCommand();
    }

    Command ramseteCommand =
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

    double dt = (System.nanoTime() - initialTime) / 1E6;
    System.out.println("RamseteCommand from file " + filename + " generation time: " + dt + "ms");


    // Run path following command, then stop at the end.
    // If told to reset odometry, reset odometry before running path.
    if(resetOdometry) {
      return new SequentialCommandGroup(
      new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
      ramseteCommand);
    }
    else {
      return ramseteCommand;
    }


  }

  public double inches2Meters(double i) {
    return Units.inchesToMeters(i);
  }

  public double feet2Meters(double feet) {
    return Units.feetToMeters(feet);
  }

}
