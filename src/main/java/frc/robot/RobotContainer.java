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
import java.util.function.BooleanSupplier;

import com.fearxzombie.limelight;
import com.team2930.lib.util.geometry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.indexerDefaultCommand;
import frc.robot.commands.indexerReverseCommand;
import frc.robot.commands.shooterAutoCommand;
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
  public static final limelight m_limelightPowerCell = new limelight("limelight");
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
    m_indexer.setDefaultCommand(new indexerDefaultCommand(m_indexer));
    m_turret.setDefaultCommand(new turretDefaultCommand(m_turret));

    chooser.addOption("AutoNav Barrel", getAutonomousBarrelCommand());
    chooser.addOption("AutoNav Slalom", getAutonomousSlalomCommand());
    chooser.addOption("AutoNav Bounce", getAutonomousBounceCommand());
    chooser.addOption("Galactic Search A", getAutonomousGalacticSearchA());
    chooser.addOption("Galactic Search B", getAutonomousGalacticSearchB());
    chooser.addOption("Galactic Search Red A", getAutonomousRedACommand());
    chooser.addOption("Galactic Search Blue A", getAutonomousBlueBCommand());
    chooser.addOption("Galactic Search Red B", getAutonomousRedBCommand());
    chooser.addOption("Galactic Search Blue B PathWeaver", getAutonomousBlueBCommand());
    chooser.addOption("Go Forward 1", autonCalibrationForward(1.0));
    chooser.addOption("Go Forward 2", autonCalibrationForward(2.0));
    chooser.addOption("Go Forward 3", autonCalibrationForward(3.0));
    chooser.addOption("Shooter Test", getAutonomousToTarget());
    chooser.addOption("Curve Left", autonCalibrationCurve(1.0, 1.0));
    chooser.addOption("Curve Right", autonCalibrationCurve(1.0, -1.0));
    chooser.setDefaultOption("Do Nothing", getNoAutonomousCommand());
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
    // A Button toggle Forza mode
    // B Button toggle square driver inputs
    // Left Trigger - reverse throttle (Forza mode)
    // Right Trigger - forward throttle (Forza mode)
    // Right Bumper - invert drive controls
    // Left Bumper - turbo boost, FULL SPEED  
    driverRightBumper.whenPressed(new InstantCommand(() -> m_drive.toggleDriveInverted()));
    // driverAButton.whenPressed(new InstantCommand(() -> m_drive.toggleForzaMode()));
    // driverBButton.whenPressed(new InstantCommand(() -> m_drive.toggleSquaredInputs()));
    // driverXButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));

    driverAButton.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake));
    driverXButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter));
    driverBackButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0), m_shooter));
    driverBButton.whenPressed(intakeReleaseCommand());

    // turn off indexo and limelight LED for drive challenges 
    driverYButton.whenPressed(new ParallelCommandGroup(
        new InstantCommand(() -> m_indexer.setStopMode()), 
        new InstantCommand(() -> m_limelight.setLEDMode(1))));


    // Operator Controls
    // Left Joystick - manual turret control
    // Left Trigger - manually move the indexer backwards
    // Right Trigger - manually move the indexer forwards
    // A Button - deploy intake
    // B Button - stop indexer
    // X Button - reverse indexer to restage balls
    // Y Button - enable normal indexer ball intake mode 
    // Right Bumper - shoot
    // Left Bumper - shoot
    // D Pad Up - manually increase ball count
    // D Pad Down - manually decrease ball count
    // Start Button - reset Odometry
    // Back Button - spool up the shooter

    //opAButton.whenPressed(new InstantCommand(() -> m_intake.deployIntake()));
    opBButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));
    //opBButton.whenPressed(new indexerStopCommand(m_indexer)); 
    //opXButton.whileHeld(new indexerReverseCommand(m_indexer));
    //opXButton.whenPressed(new InstantCommand(() -> m_hood.setPositionRotations(m_hood.angleToRotations(70.0)), m_hood));
    //opYButton.whenPressed(new InstantCommand(() -> m_hood.setPositionRotations(m_hood.angleToRotations(46.13)), m_hood));
    //opYButton.whenPressed(new InstantCommand(() -> m_indexer.setIntakeMode(), m_indexer));
    opYButton.whenPressed(intakeReleaseCommand());


    // spin up flywheel to idle RPM
    opLeftBumper.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter));
    opBackButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0), m_shooter));

    opRightBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight, m_intake));

    // opLeftBumper.whileHeld(new shooterUnderGoal(m_indexer, m_turret, m_shooter));
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
    new InstantCommand(() -> m_intake.deployIntake()), 
    new InstantCommand(() -> m_intake.setIntakePercentOutput(0.3)),
    new WaitCommand(0.3), 
    new InstantCommand(() -> m_intake.setIntakePercentOutput(-0.2)),
    new WaitCommand(0.5), 
    new InstantCommand(() -> m_intake.setIntakePercentOutput(0.0)),
    new InstantCommand(() -> m_indexer.setIntakeMode()),
    new InstantCommand(() -> m_intake.setDynamicSpeed(true))
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
        false, 1.5, 0.75);

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
   * getAutonomousBarrelCommand - generate Barrel AutoNav Command
   * 
   * @return Command object
   */
  public Command getAutonomousBarrelCommand(){

    // Tell the odometry know where the robot is starting from and what direction it is pointing.
    Pose2d startPose = new Pose2d(inches2Meters(50), inches2Meters(90), new Rotation2d(0));

    // distances are in Meters
    List<Translation2d> barrel_path_points = List.of(
      // navigate around first barrel, centered at D5 (150,60)
      new Translation2d( inches2Meters(150), inches2Meters(90)),
      new Translation2d( inches2Meters(180), inches2Meters(60)),
      new Translation2d( inches2Meters(150), inches2Meters(30)),
      new Translation2d( inches2Meters(120), inches2Meters(60)),

      // navigate around B8  (240, 120)
      new Translation2d( inches2Meters(150), inches2Meters(90)),
      new Translation2d( inches2Meters(240), inches2Meters(90)),
      new Translation2d( inches2Meters(270), inches2Meters(120)),
      new Translation2d( inches2Meters(240), inches2Meters(150)),
      new Translation2d( inches2Meters(205), inches2Meters(120)),

      // navigate around D10 (300, 60)
      new Translation2d( inches2Meters(210), inches2Meters(60)),
      new Translation2d( inches2Meters(300), inches2Meters(30)),
      new Translation2d( inches2Meters(330), inches2Meters(60)),
      new Translation2d( inches2Meters(300), inches2Meters(90)), // shift left slightly 

      // avoid cones on the way back
      new Translation2d( inches2Meters(240), inches2Meters(80)), 
      new Translation2d( inches2Meters(150), inches2Meters(100))

      // above point is just in case, but from (300, 90) to the finish is a straight shot
      );

    // Start of Barrel Path
    Command ramseteCommand = createTrajectoryCommand(
        startPose,
        barrel_path_points,
        new Pose2d(inches2Meters(60), inches2Meters(90), new Rotation2d(Math.PI)), false, 1.5, 0.50);
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousSlalomCommand - generate Barrel AutoNav Command
   * 
   * @return Command object
   */
  public Command getAutonomousSlalomCommand(){
     
    Pose2d startPose = new Pose2d(inches2Meters(45), inches2Meters(30), new Rotation2d(0));

    List<Translation2d> slalom_path_points = List.of(
      new Translation2d( inches2Meters(80), inches2Meters(30)),
      //new Translation2d( inches2Meters(90), inches2Meters(60)),
      new Translation2d( inches2Meters(105), inches2Meters(90)),
      new Translation2d( inches2Meters(255), inches2Meters(90)),
      new Translation2d( inches2Meters(270), inches2Meters(60)),

      // Between D8(240, 60) and D10(300, 60)
      new Translation2d( inches2Meters(290), inches2Meters(30)),
      new Translation2d( inches2Meters(340), inches2Meters(60)),
      new Translation2d( inches2Meters(285), inches2Meters(90)),
      //new Translation2d( inches2Meters(270), inches2Meters(60)),
      new Translation2d( inches2Meters(260), inches2Meters(30)),
      new Translation2d( inches2Meters(125), inches2Meters(30)),
      //new Translation2d( inches2Meters(90), inches2Meters(60)),
      new Translation2d( inches2Meters(75), inches2Meters(90))
      );


    // Start of The Slalom Path Program
    Command ramseteCommand = createTrajectoryCommand(
        startPose,
        slalom_path_points,
        new Pose2d(inches2Meters(60), inches2Meters(90), new Rotation2d(Math.PI)), false, 0.5, 0.25);
    
    // Run path following command, then stop at the end. Turn off Drive train
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousSlalomCommand - generate Barrel AutoNav Command
   * 
   * @return Command object
   */
  public Command getAutonomousBounceCommand(){
    Pose2d startPose = new Pose2d(inches2Meters(50), inches2Meters(90), new Rotation2d(0));    

    List<Translation2d> bounce_path_points = List.of(
      // TODO: Set up Bounce path points
      new Translation2d( inches2Meters(70), inches2Meters(90))
      // new Translation2d( inches2Meters(90), inches2Meters(150))      
      );

    // Start of The Slalom Path Program
    Command ramseteCommand = createTrajectoryCommand(
        startPose,
        bounce_path_points,
        new Pose2d(inches2Meters(90), inches2Meters(150), new Rotation2d(Math.PI/2)), false, 1.0, 0.25);
    
    // Run path following command, then stop at the end. Turn off Drive train
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  
  /**
   * getAutonomousBlueBCommand - generate Blue B AutoNav Command
   * @return Command object
   */
  public Command getAutonomousBlueBCommand(){

    return new ParallelCommandGroup(
      intakeReleaseCommand(),
      loadPathWeaverTrajectoryCommand("paths/BlueB.wpilib.json")
    );

  }

  /**
   * getAutonomousBlueACommand - generate Blue A AutoNav Command
   * @return Command object
   */
  public Command getAutonomousBlueACommand(){
    //Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));

    List<Translation2d> blue_a_points = List.of(
      new Translation2d( inches2Meters(180), inches2Meters(30)),
      new Translation2d( inches2Meters(210), inches2Meters(120)),
      new Translation2d( inches2Meters(270), inches2Meters(90))
      );
    
    // Start of Blue A program
    Command ramseteCommand = createTrajectoryCommand(
      startPose,
      blue_a_points,
      new Pose2d(inches2Meters(340), inches2Meters(90), new Rotation2d(0)), false, 1.0, 0.25
    );

    // Run path following command, then stop at end. Turn off Drive train
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousRedBCommand - generate Red B AutoNav Command
   * @return Command object
   */
  public Command getAutonomousRedBCommand(){
    // Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));

    List<Translation2d> red_b_points = List.of(
      new Translation2d( inches2Meters(90), inches2Meters(120)),
      new Translation2d( inches2Meters(150), inches2Meters(60)),
      new Translation2d( inches2Meters(210), inches2Meters(120))
      );
    
    // Start of Red B program
    Command ramseteCommand = createTrajectoryCommand(
      startPose,
      red_b_points,
      new Pose2d(inches2Meters(320), inches2Meters(90), new Rotation2d(0)), false, 1.0, 0.25
    );

    // Run path following command, then stop at end. Turn off Drive train
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousRedACommand - generate Red A AutoNav Command
   * @return Command object
   */
  public Command getAutonomousRedACommand(){
    //Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));

    List<Translation2d> red_a_points = List.of(
      new Translation2d( inches2Meters(90), inches2Meters(85)),
      new Translation2d( inches2Meters(150), inches2Meters(75)),
      new Translation2d( inches2Meters(180), inches2Meters(145))
      );
    
    // Start of Red A program
    Command ramseteCommand = createTrajectoryCommand(
      startPose,
      red_a_points,
      new Pose2d(inches2Meters(320), inches2Meters(95), new Rotation2d(0)), false, 1.5, 0.5
    );

    // Run path following command, then stop at end. Turn off Drive train
    return ramseteCommand;
  }

  /**
   * getAutonomousGalacticSearchA - Generate Auton Command used in Galactic Search A Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousGalacticSearchA() {
    Command RedA = getAutonomousRedACommand();
    Command BlueA = getAutonomousBlueACommand();

    BooleanSupplier seesPowerCell = () -> (m_limelightPowerCell.getTV() == 1.0);

    // If it sees the Power Cell, we run Red A, if not, then we run Blue A
    return new ParallelCommandGroup(
      intakeReleaseCommand(),
      new ConditionalCommand(RedA, BlueA, seesPowerCell));

  }

  /**
   * getAutonomousGalacticSearchB - Generate Auton Command used in Galactic Search B Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousGalacticSearchB() {
    Command RedB = getAutonomousRedBCommand();
    Command BlueB = getAutonomousBlueBCommand();

    BooleanSupplier seesPowerCell = () -> (m_limelightPowerCell.getTV() == 1.0);

    // If it sees the Power Cell, we run Red B, if not, then we run Blue B
    return new ParallelCommandGroup(
      intakeReleaseCommand(),
      new ConditionalCommand(RedB, BlueB, seesPowerCell));
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
   * @return RamsetCommand
   */
  public Command loadPathWeaverTrajectoryCommand(String filename) {

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
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
        ramseteCommand);


  }

  // TODO: this should be in com/team2930/utils/units.java or a new Units.java under Utils
  public double inches2Meters(double i) {
    return i * 0.0254;
  }

  public double feet2Meters(double feet) {
    return (feet * 0.3048);
  }

}
