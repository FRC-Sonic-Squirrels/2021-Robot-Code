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

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;

  private RobotContainer m_robotContainer;
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  SendableChooser <String> chooser = new SendableChooser<>();
  public static double distance_meters = 0.0;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  
    SmartDashboard.putNumber("distance ft", 0);
    //RobotContainer.m_limelight.setLEDMode(1);
    CameraServer.getInstance().startAutomaticCapture();
    chooser.addOption("AutoNav Barrel", "barrel");
    chooser.addOption("AutoNav Slalom", "slalom");
    chooser.addOption("AutoNav Bounce", "bounce");
    // TODO: put galactic search auton choices here
    chooser.setDefaultOption("Do Nothing", "donothing");
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

    String autoName = chooser.getSelected();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoName);

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
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
