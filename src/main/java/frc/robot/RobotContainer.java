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
import com.team2930.lib.util.geometry;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
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
import frc.robot.commands.indexerStopCommand;
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
  public Translation2d powerPortLocation = new Translation2d(geometry.feet2Meters(10), 0);

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
    driverAButton.whenPressed(new InstantCommand(() -> m_drive.toggleForzaMode()));
    driverBButton.whenPressed(new InstantCommand(() -> m_drive.toggleSquaredInputs()));
    
    // driverXButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));

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

    opAButton.whenPressed(new InstantCommand(() -> m_intake.deployIntake()));
    opBButton.whenPressed(new InstantCommand(() -> m_intake.toggleDynamicMode()));
    //opBButton.whenPressed(new indexerStopCommand(m_indexer)); 
    opXButton.whileHeld(new indexerReverseCommand(m_indexer));
    opYButton.whenPressed(new InstantCommand(() -> m_indexer.setIntakeMode(), m_indexer));

    // spin up flywheel to idle RPM
    opLeftBumper.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(3000), m_shooter));
    opBackButton.whenPressed(new InstantCommand(() -> m_shooter.setShooterRPM(0)));

    opRightBumper.whileHeld(new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight));

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
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoName) {

    // zero gyro heading and reset encoders to zero.
    m_drive.zeroHeading();
    m_drive.resetEncoders();

    if (autoName == "donothing") {
      return getNoAutonomousCommand();
    }
    else if (autoName == "figure8") {
      return getAutonomousFigure8Command();
    }
    else if (autoName == "barrel") {
      return getAutonomousBarrelCommand();
    }
    else if (autoName == "slalom") {
      return getAutonomousSlalomCommand();
    }
    else if (autoName == "bounce") {
      return getAutonomousBounceCommand();
    }
    else if(autoName == "galacticSearchA"){
      return getAutonomousGalacticSearchA();
    }
    else if(autoName == "galacticSearchB"){
      return getAutonomousGalacticSearchB();
    }
    else if (autoName == "reda") {
      return getAutonomousRedACommand();
    }
    else if (autoName == "redb") {
      return getAutonomousRedBCommand();
    }
    else if (autoName == "bluea") {
      return getAutonomousBlueACommand();
    }
    else if (autoName == "blueb") {
      return getAutonomousBlueBCommand();
    }
    else if (autoName == "forward1") {
      return autonCalibrationForward(1.0);
    }
    else if (autoName == "forward2") {
      return autonCalibrationForward(2.0);
    }
    else if (autoName == "forward3") {
      return autonCalibrationForward(3.0);
    }
    else if (autoName == "curveLeft") {
      return autonCalibrationCurve(1.0, 1.0);
    }   
    else if (autoName == "curveRight") {
      return autonCalibrationCurve(1.0, -1.0);
    }
    else if(autoName == "shooterTest"){
      return getAutonomousToTarget();
    }
    else if (autoName == "nothing") {
      return getNoAutonomousCommand();
    }
 
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

    m_drive.resetOdometry(startPose);

    RamseteCommand ramseteCommand = createTrajectoryCommand(
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

    m_drive.resetOdometry(startPose);

    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
    m_drive.resetOdometry(startPose);

    // TODO: complete Barrel Auton command and uncomment the next line

    // distances are in Meters
    List<Translation2d> barrel_path_points = List.of(
      // navigate around first barrel, centered at D5 (150,60)
      new Translation2d( inches2Meters(150), inches2Meters(90)),
      new Translation2d( inches2Meters(180), inches2Meters(60)),
      new Translation2d( inches2Meters(150), inches2Meters(30)),
      new Translation2d( inches2Meters(120), inches2Meters(60)),

      // TODO: navigate around B8  (240, 120)
      new Translation2d( inches2Meters(150), inches2Meters(90)),
      new Translation2d( inches2Meters(240), inches2Meters(90)),
      new Translation2d( inches2Meters(270), inches2Meters(120)),
      new Translation2d( inches2Meters(240), inches2Meters(150)),
      new Translation2d( inches2Meters(205), inches2Meters(120)),

      // TODO: navigate around D10 (300, 60)
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
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
    m_drive.resetOdometry(startPose);

    List<Translation2d> slalom_path_points = List.of(
      // TODO: Set up Slalom path points
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
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
    m_drive.resetOdometry(startPose);
    

    List<Translation2d> bounce_path_points = List.of(
      // TODO: Set up Slalom path points
      new Translation2d( inches2Meters(70), inches2Meters(90))
      // new Translation2d( inches2Meters(90), inches2Meters(150))      
      );

    // Start of The Slalom Path Program
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
    //Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));
    m_drive.resetOdometry(startPose);


    List<Translation2d> blue_b_points = List.of(
      new Translation2d( inches2Meters(180), inches2Meters(60)),
      new Translation2d( inches2Meters(240), inches2Meters(120)),
      new Translation2d( inches2Meters(300), inches2Meters(60))
      );
    
    // Start of Blue B program
    RamseteCommand ramseteCommand = createTrajectoryCommand(
      startPose,
      blue_b_points,
      new Pose2d(inches2Meters(340), inches2Meters(90), new Rotation2d(0)), false, 1.0, 0.25
    );

    // Run path following command, then stop at end. Turn off Drive train
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }


  /**
   * getAutonomousBlueACommand - generate Blue A AutoNav Command
   * @return Command object
   */
  public Command getAutonomousBlueACommand(){
    //Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));
    m_drive.resetOdometry(startPose);


    List<Translation2d> blue_a_points = List.of(
      new Translation2d( inches2Meters(180), inches2Meters(30)),
      new Translation2d( inches2Meters(210), inches2Meters(120)),
      new Translation2d( inches2Meters(270), inches2Meters(90))
      );
    
    // Start of Blue A program
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
      //Changing start pose to 12 by 90 because of Orange 1 size
    Pose2d startPose = new Pose2d(inches2Meters(12), inches2Meters(90), new Rotation2d(0));
    m_drive.resetOdometry(startPose);


    List<Translation2d> red_b_points = List.of(
      new Translation2d( inches2Meters(90), inches2Meters(120)),
      new Translation2d( inches2Meters(150), inches2Meters(60)),
      new Translation2d( inches2Meters(210), inches2Meters(120))
      );
    
    // Start of Red B program
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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
    m_drive.resetOdometry(startPose);

    List<Translation2d> red_a_points = List.of(
      new Translation2d( inches2Meters(90), inches2Meters(85)),
      new Translation2d( inches2Meters(150), inches2Meters(75)),
      new Translation2d( inches2Meters(180), inches2Meters(145))
      );
    
    // Start of Red A program
    RamseteCommand ramseteCommand = createTrajectoryCommand(
      startPose,
      red_a_points,
      new Pose2d(inches2Meters(320), inches2Meters(95), new Rotation2d(0)), false, 1.5, 0.5
    );

    // Run path following command, then stop at end. Turn off Drive train
    return ramseteCommand;
  }

  /**
   * getAutonomousGalacicSearchA - Generate Auton Command used in Galactic Search A Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousGalacticSearchA() {
    Command RedA = getAutonomousRedACommand();
    Command BlueA = getAutonomousBlueACommand();

    Command intakeStart = new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.deployIntake()), 
      new WaitCommand(0.7), 
      new InstantCommand(() -> m_indexer.setIntakeMode()),
      new InstantCommand(() -> m_intake.setDynamicSpeed(true))
    );

    //If it sees the powercell, we run Red A, if not, then we run Blue A
    return new ConditionalCommand(new ParallelCommandGroup(intakeStart, RedA), new ParallelCommandGroup(intakeStart, BlueA), m_intake::seesPowerCell);
  }

  /**
   * getAutonomousGalacicSearchB - Generate Auton Command used in Galactic Search B Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousGalacticSearchB() {
    Command RedB = getAutonomousRedBCommand();
    Command BlueB = getAutonomousBlueBCommand();

    Command intakeStart = new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.deployIntake()), 
      new WaitCommand(0.7), 
      new InstantCommand(() -> m_indexer.setIntakeMode()),
      new InstantCommand(() -> m_intake.setDynamicSpeed(true))
    );

    if(m_limelightPowerCell.getTX() != 0.0){
      return new ParallelCommandGroup(intakeStart, RedB);
    }
    else {
      return new ParallelCommandGroup(intakeStart, BlueB);
    }
  }


  /**
   * getAutonomousToTarget - Generate Auton Command used in Autonomous Challenge
   * 
   * @return Command object
   */
  public Command getAutonomousToTarget(){
    //Start of Shooting with Red A
    Pose2d startPose = new Pose2d(inches2Meters(320), inches2Meters(95), new Rotation2d(0));

    RamseteCommand ramseteCommand = createTrajectoryCommand(
      startPose,
      List.of(),
      new Pose2d(inches2Meters(90), inches2Meters(160), new Rotation2d(0)), true, 1.5, 0.5
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.setDynamicSpeed(false)),
      ramseteCommand, 
      new InstantCommand(() -> m_drive.tankDriveVolts(0, 0)),
      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight).withTimeout(15),
      new shooterAutoCommand(m_indexer, m_turret, m_shooter, m_hood, m_limelight).withTimeout(5),
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
    RamseteCommand ramseteCommand = createTrajectoryCommand(
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

    long initialTime = System.nanoTime();

    // trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
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

    double dt = (System.nanoTime() - initialTime) / 1E6;
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
