package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up.
   */
  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs every 20ms in all modes.
   * Required for CommandScheduler to function.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.wheelSetup();
  }

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
