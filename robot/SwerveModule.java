package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants.*;

//The class is used to represent a drive talon and turn talon that are part of one the swerve modules
public class SwerveModule {
    //The Neo motors
    public CANSparkMax driveMotor;
    public CANSparkMax turnMotor;

    private RelativeEncoder driveRelative;

    //The encoder for the turn motor
    private AnalogEncoder turnEncoder;
    private double encoderOffset;

    private int encoderInvert;

    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, boolean driveMotorInvert, boolean turnMotorInvert, boolean encoderInvert, double offsetAngle) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveRelative = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnEncoderID);

        encoderOffset = offsetAngle;
        driveMotor.setInverted(driveMotorInvert);
        turnMotor.setInverted(turnMotorInvert);

        this.encoderInvert = encoderInvert ? -1 : 1;
    }

    //Returns a SwerveModuleState representation of this SwerveModule
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteTurnPosition()));
    }

    public void set(double driveSet, double turnSet) {
        driveMotor.set(driveSet);
        turnMotor.set(turnSet);
    }

    //Metres position of the drive talon
    public double getDrivePosition() {
        return driveRelative.getPosition() * driveConstants.metresPerRotation;
    }

    //Radians position of the turn talon
    public double getAbsoluteTurnPosition() {
        //SmartDashboard.putNumber("Raw Position " + Constants.counter + ": ", (turnEncoder.getAbsolutePosition() - encoderOffset));
        //NEW, degree calculations + removed the -180 (or pi) from the end
        double temp = 180 - encoderInvert * (turnEncoder.getAbsolutePosition() - encoderOffset) * turnConstants.degreesPerRotation;

        while(temp <= 0) {
            temp += 360;
        }
        while(temp > 360) {
            temp -= 360;
        }

        /*SmartDashboard.putNumber("Temp Position " + Constants.counter++ + ": ", temp);

        if(Constants.counter == 4) {
            Constants.counter = 0;
        }*/

        return temp;
    }

    //Metres/second velocity of the drive talon
    public double getDriveVelocity() {
        return driveRelative.getVelocity() * driveConstants.metresPerRotation / 60.0d;
    }
}
