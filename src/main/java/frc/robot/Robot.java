package frc.robot;

import static frc.robot.Constants.limeLightConstants.targetHeight_meters;
import static frc.robot.Constants.limeLightConstants.limeLightHeight_meters;
import static frc.robot.Constants.limeLightConstants.limeLightAngle_degrees;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel(0);
  public static double distance_meters = 0.0;

  private double turretErrorDeg = 0.0;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    LiveWindow.disableAllTelemetry();
    
    //RobotContainer.m_limelight.setLEDMode(1);
    //CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (RobotContainer.m_limelight != null) {
      if (RobotContainer.m_limelight.getTV() == 1.0) {
        turretErrorDeg = RobotContainer.m_limelight.getTX();
        if (Math.abs(turretErrorDeg) < 0.75) {
          RobotContainer.limelightOnTarget = true;
        } else {
          RobotContainer.limelightOnTarget = false;
        }
        distance_meters = RobotContainer.m_limelight.getDist(targetHeight_meters, limeLightHeight_meters , limeLightAngle_degrees);
        SmartDashboard.putNumber("distance ft", distance_meters * 3.28084);
      }
      else {
        RobotContainer.limelightOnTarget = false;
      }
    }

    SmartDashboard.putBoolean("limelight on target", RobotContainer.limelightOnTarget);
    SmartDashboard.putNumber("turret Error Deg", turretErrorDeg);
    SmartDashboard.putBoolean("Sees PowerCell", (RobotContainer.m_limelightPowerCell.getTV() == 1.0));

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
    m_autonomousCommand = m_robotContainer.chooser.getSelected();

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
