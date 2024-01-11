package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
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
  private CANSparkMax testMotor;
  private RelativeEncoder testEncoder;
  private CANSparkMax testTurnMotor;
  private RelativeEncoder testTurnEncoder;
  private double maxRPM;
  private AnalogEncoder encoder;
  
  @Override
  public void robotInit() {
    SmartDashboard.putString("Current mode: ", "robotInit");

    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 1d;
    //mySwerveMaster = new SwerveMaster();
    testMotor = new CANSparkMax(2, MotorType.kBrushless);
    testTurnMotor = new CANSparkMax(6, MotorType.kBrushless);
    testEncoder = testMotor.getEncoder();
    testTurnEncoder = testTurnMotor.getEncoder();
    maxRPM = 0d;
    encoder = new AnalogEncoder(3);
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

    //mySwerveMaster.update(driveController, driveControllerFactor);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    //mySwerveMaster.set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
    testMotor.set(0);
    testTurnMotor.set(0);
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

    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right X: ", driveController.getRightX());

    testMotor.set(driveControllerFactor * (Math.abs(driveController.getLeftY()) < Constants.driveControllerStopBelowThis ? 0d : driveController.getLeftY()));
    testTurnMotor.set(driveControllerFactor * (Math.abs(driveController.getRightX()) < Constants.driveControllerStopBelowThis ? 0d : driveController.getRightX()));

    if(Math.abs(testEncoder.getVelocity()) > maxRPM) {
      maxRPM = Math.abs(testEncoder.getVelocity());
      SmartDashboard.putNumber("Max RPM: ", maxRPM);
    }

    SmartDashboard.putNumber("Drive RPM: ", testEncoder.getVelocity());
    SmartDashboard.putNumber("Turn RPM: ", testTurnEncoder.getVelocity());
    SmartDashboard.putNumber("Encoder value: ", encoder.getAbsolutePosition());
  }
}