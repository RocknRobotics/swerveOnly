package frc.robot;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants.turnConstants;

public class Robot extends TimedRobot {
  //The PS4Controller for driving
  private PS4Controller driveController;
  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1 (basically different speed levels)
  public static double driveControllerFactor;
  //Turn factor
  public static double turnFactor;
  //Self-explanatory
  private SwerveMaster mySwerveMaster;
  private DutyCycleEncoder testDutyCycleEncoder;
  
  @Override
  public void robotInit() {
    SmartDashboard.putString("Current mode: ", "robotInit");

    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 0.25d;
    turnFactor = 0.25d;
    mySwerveMaster = new SwerveMaster();

    mySwerveMaster.leftUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.leftDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.resetAccelerometer();

    testDutyCycleEncoder = new DutyCycleEncoder(0);

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
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Mode: ", "teleopPeriodic");

    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getSquareButtonPressed()) {
      driveControllerFactor = 0.25d;
    } else if(driveController.getCrossButtonPressed()) {
      driveControllerFactor = 0.5d;
    } else if(driveController.getCircleButtonPressed()) {
      driveControllerFactor = 0.75d;
    } else if(driveController.getTriangleButtonPressed()) {
      driveControllerFactor = 1.0d;
    }

    if(driveController.getPSButtonPressed()) {
      turnFactor = 0d;
    } else if(driveController.getPOV() == 270) {
      turnFactor = 0.25d;
    } else if(driveController.getPOV() == 180) {
      turnFactor = 0.5d;
    } else if(driveController.getPOV() == 90) {
      turnFactor = 0.75d;
    } else if(driveController.getPOV() == 0) {
      turnFactor = 1.0d;
    }

    if (driveController.getShareButtonPressed()) {
      mySwerveMaster.resetOrigin();
    }

    //NEW, since accelerometer will need to be reset due to inaccuracies accumulating
    if(driveController.getOptionsButtonPressed()) {
      mySwerveMaster.resetAccelerometer();
    }

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Turn Factor: ", turnFactor);
    SmartDashboard.putNumber("DC Left X: ", driveController.getLeftX());    
    SmartDashboard.putNumber("DC Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("DC Right X: ", driveController.getRightX());

    mySwerveMaster.update(driveController, driveControllerFactor, turnFactor);
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Mode: ", "disabledInit");
    mySwerveMaster.set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    SmartDashboard.putBoolean("DIO Encoder Connected: ", testDutyCycleEncoder.isConnected());
    SmartDashboard.putNumber("DIO Encoder: ", testDutyCycleEncoder.getAbsolutePosition());
  }
}