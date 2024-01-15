package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //The PS4Controller for driving
  private PS4Controller driveController;
  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1 (basically different speed levels)
  private double driveControllerFactor;
  //Self-explanatory
  private SwerveMaster mySwerveMaster;
  
  //Testing stuff
  double maxSpeed = 0.0;
  double time = 0.0;
  double prevAngle = 0.0;
  
  @Override
  public void robotInit() {
    SmartDashboard.putString("Current mode: ", "robotInit");

    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 1d;
    mySwerveMaster = new SwerveMaster();

    mySwerveMaster.leftUpModule.driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.leftDownModule.driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.rightUpModule.driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.rightDownModule.driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.resetAccelerometer();
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Current mode: ", "teleopInit");
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Current mode: ", "teleopPeriodic");

    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getSquareButtonPressed()) {
      driveControllerFactor = 0.25d;
    } else if(driveController.getCrossButtonPressed()) {
      driveControllerFactor = 0.5d;
    } else if(driveController.getCircleButtonPressed()) {
      driveControllerFactor = 0.75d;
    } else if(driveController.getTriangleButtonPressed()) {
      driveControllerFactor = 1d;
    }

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right X: ", driveController.getRightX());
    SmartDashboard.putNumber("Left Up Encoder: ", mySwerveMaster.leftUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Left Down Encoder: ", mySwerveMaster.leftDownModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Up Encoder: ", mySwerveMaster.rightUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Down Encoder: ", mySwerveMaster.rightDownModule.getAbsoluteTurnPosition());

    mySwerveMaster.update(driveController, driveControllerFactor);
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Current mode: ", "disabledInit");
    mySwerveMaster.set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
