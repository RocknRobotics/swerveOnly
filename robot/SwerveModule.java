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

    private double prevTime;
    private double[] currPos;

    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, boolean driveMotorInvert, boolean turnMotorInvert, boolean encoderInvert, double offsetAngle) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveRelative = driveMotor.getEncoder();
        driveRelative.setVelocityConversionFactor(1);
        turnEncoder = new AnalogEncoder(turnEncoderID);

        encoderOffset = offsetAngle;
        driveMotor.setInverted(driveMotorInvert);
        turnMotor.setInverted(turnMotorInvert);

        prevTime = System.currentTimeMillis();
        currPos = new double[2];

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

    //Degrees position of the turn talon
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
        return driveRelative.getVelocity() * driveConstants.metresPerRotation / 60d;
    }

    //Set x and y position of the motor in meters. 
    public void resetPosition(double x, double y) {
        currPos[0] = x;
        currPos[1] = y;
    }

    //Get field position of the motor relative to the start origin
    public double[] getPosition(double reducedAngle) {
        //Get the field relative angle that the wheel is pointing
        double moveAngle = getAbsoluteTurnPosition() - reducedAngle;
        while (moveAngle < 0) {
            moveAngle += 360;
        }

        //Get how far the wheel moved in that direction
        //Velocity is reversed to make x positive going right and y positive going forward
        double currTime = System.currentTimeMillis();
        double moveLength = -getDriveVelocity() * ((currTime - prevTime) / 1000d);

        
        /*SmartDashboard.putNumber("M0 Angle", moveAngle);
        SmartDashboard.putNumber("M0 moveLength", moveLength);
        SmartDashboard.putNumber("M0 velocity", getDriveVelocity());
        SmartDashboard.putNumber("M0 rpm", driveRelative.getVelocity());
        SmartDashboard.putNumber("M0 timeP", prevTime);
        SmartDashboard.putNumber("M0 timeC", currTime);
        SmartDashboard.putNumber("M0 timechange", (currTime - prevTime));
        */

        prevTime = currTime;

        //Calculate how x and y are related using the angle
        double factorY = Math.cos(moveAngle * Math.PI / 180);
        double factorX = Math.sin(moveAngle * Math.PI / 180);

        //Multiply factors by distance travelled to get x and y changes.
        double deltaY = factorY * moveLength;
        double deltaX = factorX * moveLength;

        //Update currX and currY
        currPos[0] += deltaX;
        currPos[1] += deltaY;

        //Return position
        return currPos;
    }
}
