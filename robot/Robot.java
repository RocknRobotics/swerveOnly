package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants.turnConstants;

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

    mySwerveMaster.swerveModules[0].driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.swerveModules[1].driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.swerveModules[2].driveMotor.setIdleMode(IdleMode.kBrake);
    mySwerveMaster.swerveModules[3].driveMotor.setIdleMode(IdleMode.kBrake);
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
    SmartDashboard.putNumber("Left Up Encoder: ", mySwerveMaster.swerveModules[0].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Left Down Encoder: ", mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Up Encoder: ", mySwerveMaster.swerveModules[2].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Down Encoder: ", mySwerveMaster.swerveModules[3].getAbsoluteTurnPosition());

    mySwerveMaster.update(driveController, driveControllerFactor);
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Current mode: ", "disabledInit");
    mySwerveMaster.disable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    SmartDashboard.putString("Current mode: ", "testInit");
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putString("Current mode: ", "testPeriodic");

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

    double currSpeed = 0.0;

    if(time == 0.0) {
      prevAngle = mySwerveMaster.accelerometer.getYaw();
      time = System.nanoTime();
    } else {
      double tempAngle = mySwerveMaster.accelerometer.getYaw();
      currSpeed = Math.abs(mySwerveMaster.accelerometer.getYaw() - prevAngle) / ((System.nanoTime() - time) * Math.pow(10, -9));
      if(tempAngle > 300 && prevAngle < 100) {
        currSpeed = 0.0;
      }
      if(tempAngle < 100 && prevAngle > 300) {
        currSpeed = 0.0;
      }
      if(currSpeed > 10000) {
        currSpeed = 0.0;
      }
      prevAngle = mySwerveMaster.accelerometer.getYaw();
      time = System.nanoTime();
    }

    if(currSpeed > maxSpeed) {
      maxSpeed = currSpeed;
    }

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right X: ", driveController.getRightX());
    SmartDashboard.putNumber("Max Speed: ", maxSpeed);
    SmartDashboard.putNumber("Left Up Encoder: ", mySwerveMaster.swerveModules[0].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Left Down Encoder: ", mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Up Encoder: ", mySwerveMaster.swerveModules[2].getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Down Encoder: ", mySwerveMaster.swerveModules[3].getAbsoluteTurnPosition());


    double leftY = Math.abs(driveController.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : driveController.getLeftY() * driveControllerFactor;
    //double rightX = Math.abs(driveController.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : driveController.getRightX() * driveControllerFactor;

    double[] turnSets = new double[]{0.05 * (7 * Math.PI / 4 - mySwerveMaster.swerveModules[0].getAbsoluteTurnPosition()), 
      0.025 * (Math.PI / 4 - (mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition() < 0 ? mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition() + Math.PI : mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition())), 
      0.05 * (5 * Math.PI / 4 - mySwerveMaster.swerveModules[2].getAbsoluteTurnPosition()), 
      0.05 * (3 * Math.PI / 4 - mySwerveMaster.swerveModules[3].getAbsoluteTurnPosition())};

      if(mySwerveMaster.swerveModules[0].getAbsoluteTurnPosition() < Math.PI) {
        turnSets[0] *= -1;
      }
      if(mySwerveMaster.swerveModules[1].getAbsoluteTurnPosition() < Math.PI) {
        turnSets[1] *= -1;
      }
      if(mySwerveMaster.swerveModules[2].getAbsoluteTurnPosition() < Math.PI) {
        turnSets[2] *= -1;
      }
      if(mySwerveMaster.swerveModules[3].getAbsoluteTurnPosition() < Math.PI) {
        turnSets[3] *= -1;
      }

      turnSets[0] = 0.0;
      turnSets[1] = 0.0;
      turnSets[2] = 0.0;
      turnSets[3] = 0.0;

    //mySwerveMaster.set(new double[]{leftY, leftY, leftY, leftY}, turnSets);
  }
}
