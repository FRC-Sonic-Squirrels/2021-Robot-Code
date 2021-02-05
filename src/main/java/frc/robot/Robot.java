package frc.robot;

import static frc.robot.Constants.limeLightConstants.targetHeight_meters;
import static frc.robot.Constants.limeLightConstants.limeLightHeight_meters;
import static frc.robot.Constants.limeLightConstants.limeLightAngle_degrees;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public char stage2ColorChar = 'U';
  private RobotContainer m_robotContainer;
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  public Compressor Compressor;
  //public static boolean isCompBot = true;
  SendableChooser <String> chooser = new SendableChooser<>();
  public static double distance_meters = 0.0;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  
    SmartDashboard.putNumber("distance ft", 0);
    //RobotContainer.m_limelight.setLEDMode(1);
    CameraServer.getInstance().startAutomaticCapture();
    chooser.addOption("Right 3 Ball", "r3");
    chooser.addOption("Right 4 ball", "r4");
    chooser.addOption("Right 5 ball", "r5");
    chooser.addOption("Right 6 ball", "r6");
    chooser.addOption("Right 6 ball test", "r6t");
    chooser.addOption("straight on 3", "s3");
    chooser.addOption("straight on 3 forward", "s3f");
    chooser.addOption("right side 6 reorg", "r6reorg");
    chooser.setDefaultOption("Center 3 ball", "m3");
    SmartDashboard.putData("Auto mode", chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("limelight on target", RobotContainer.limelightOnTarget);
    if (RobotContainer.m_limelight.getTV() == 1 ) {
      // limelight can see a target
      distance_meters = RobotContainer.m_limelight.getDist(targetHeight_meters, limeLightHeight_meters , limeLightAngle_degrees);
      // distance_meters = ( 2.5019 - 0.603250) / Math.tan( Math.toRadians(30 + RobotContainer.m_limelight.getTY()));
      SmartDashboard.putNumber("distance ft", distance_meters * 3.28084);
    }
  }

  @Override
  public void disabledInit() {
    //RobotContainer.m_limelight.setLEDMode(1);
    m_robotContainer.m_shooter.setShooterRPM(0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // TODO: move this to RobotInit() that gets run when the robot is powered on instead of here when
    // Autonomous starts. Auton command generation can take almost a second. Don't waste it during a 
    // match.
    if (chooser.getSelected() == "r3"){
      m_autonomousCommand = m_robotContainer.rightSide3Ball();
    }
    if (chooser.getSelected() == "r4"){
      m_autonomousCommand = m_robotContainer.rightSide4Ball();
    }
    if (chooser.getSelected() == "r5"){
      m_autonomousCommand = m_robotContainer.rightSide5Ball();
    }
    if (chooser.getSelected() == "m3"){
      m_autonomousCommand = m_robotContainer.middle3Ball();
    }
    if (chooser.getSelected() == "s3"){
      m_autonomousCommand = m_robotContainer.straightOn3Ball();
    }
    if (chooser.getSelected() == "s3_reorg"){
      m_autonomousCommand = m_robotContainer.straightOn3Ball_reorg();
    }
    if (chooser.getSelected() == "r6"){
      m_autonomousCommand = m_robotContainer.rightSide6Ball();
    }
    if (chooser.getSelected() == "r6t"){
      m_autonomousCommand = m_robotContainer.rightSide6BallTest();
    }
    if (chooser.getSelected() == "r6reorg"){
      m_autonomousCommand = m_robotContainer.rightSide6Ball_reorg();
    }
    if (chooser.getSelected() == "s3f") {
      m_autonomousCommand = m_robotContainer.straightOn3BallForward();
    }

    if (m_autonomousCommand != null) {
      System.out.println("Scheduling Autonomous Command");
      m_autonomousCommand.schedule();
      System.out.println("Finished Autonomous Command");
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      System.out.println("Cancelling Autonomous Command");
      m_autonomousCommand.cancel();
      // TODO: each of those commands is going to re-create a new command, the ones with trajectories will take a non-trivial amount of time
      // maybe create the them in RobotContainer and save them to a variable?
    }
  }

  @Override
  public void teleopPeriodic() {
    //Does the target color also have to be a number 
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        // Blue case code
       stage2ColorChar = 'B';
        break;
      case 'G':
        // Green case code
       stage2ColorChar = 'G';
        break;
      case 'R':
        // Red case code
        stage2ColorChar = 'R';
        break;
      case 'Y':
        // Yellow case code
        stage2ColorChar = 'Y';
        break;
      default:
        // This is corrupt data
        break;
      }
    } else {
      // Code for no data received yet
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}