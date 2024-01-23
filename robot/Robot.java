package frc.robot;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants.turnConstants;

public class Robot extends TimedRobot {
  //The PS4Controller for driving
  private PS4Controller driveController;
  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1 (basically different speed levels)
  public static double driveControllerFactor;
  //Self-explanatory
  private SwerveMaster mySwerveMaster;
  
  //Testing stuff
  double maxSpeed = 0.0;
  
  @Override
  public void robotInit() {
    SmartDashboard.putString("Current mode: ", "robotInit");

    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 1d;
    mySwerveMaster = new SwerveMaster();

    mySwerveMaster.leftUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.leftDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.resetAccelerometer();

    SmartDashboard.putNumber("kP: ", 0.8);
    SmartDashboard.putNumber("kI: ", 0.0);
    SmartDashboard.putNumber("kD: ", 0.0);
    SmartDashboard.putNumber("Poition Tolerance: ", 0.1);
    SmartDashboard.putNumber("Velocity Tolerance: ", 0.01);
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

    //NEW, since accelerometer will need to be reset due to inaccuracies accumulating
    if(driveController.getOptionsButtonPressed()) {
      mySwerveMaster.resetAccelerometer();
    }

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right X: ", driveController.getRightX());

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

    double currSpeed = Math.sqrt(Math.pow(mySwerveMaster.accelerometer.getVelocityX(), 2) + Math.pow(mySwerveMaster.accelerometer.getVelocityY(), 2) + Math.pow(mySwerveMaster.accelerometer.getVelocityZ(), 2));

    if(currSpeed > maxSpeed) {
      maxSpeed = currSpeed;
    }

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right X: ", driveController.getRightX());
    SmartDashboard.putNumber("Max Speed: ", maxSpeed);
    SmartDashboard.putNumber("Left Up Encoder: ", mySwerveMaster.leftUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Left Down Encoder: ", mySwerveMaster.leftDownModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Up Encoder: ", mySwerveMaster.rightUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Right Down Encoder: ", mySwerveMaster.rightDownModule.getAbsoluteTurnPosition());


    double leftY = Math.abs(driveController.getLeftY()) < Constants.driveControllerStopBelowThis ? 0.0 : driveController.getLeftY() * driveControllerFactor;
    //double rightX = Math.abs(driveController.getRightX()) < Constants.driveControllerStopBelowThis ? 0.0 : driveController.getRightX() * driveControllerFactor;

    double[] turnSets = new double[]{0.05 * (turnConstants.leftUpOffset - mySwerveMaster.leftUpModule.getAbsoluteTurnPosition()), 
      0.05 * (turnConstants.leftDownOffset- mySwerveMaster.leftDownModule.getAbsoluteTurnPosition()), 
      0.05 * (turnConstants.rightUpOffset - mySwerveMaster.rightUpModule.getAbsoluteTurnPosition()), 
      0.05 * (turnConstants.rightDownOffset - mySwerveMaster.rightDownModule.getAbsoluteTurnPosition())};
    
      for(int i = 0; i < turnSets.length; i++) {
        if(Math.abs(turnSets[i]) < 0.01) {
          turnSets[i] = 0.0;
        }
      }


    mySwerveMaster.set(new double[]{leftY, leftY, leftY, leftY}, turnSets);
  }
}