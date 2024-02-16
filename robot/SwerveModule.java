package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.motorConstants.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//The class is used to represent a drive talon and turn talon that are part of one the swerve modules
public class SwerveModule {
    //The Neo motors
    public CANSparkMax driveMotor;
    public CANSparkMax turnMotor;

    //Relative encoder built into motor controller for odometry
    private RelativeEncoder driveRelative;

    //The absolute encoder for the turn motor
    private AnalogEncoder turnEncoder;
    private double encoderOffset;

    //Encoder invert
    private int encoderInvert;

    //Odometry
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

    //Set drive and turn motors
    public void set(double driveSet, double turnSet) {
        driveMotor.set(driveSet);
        turnMotor.set(turnSet);
    }

    //Metres position of the drive talon
    public double getDrivePosition() {
        return driveRelative.getPosition() * driveConstants.metresPerRotation;
    }

    // Degrees position of the turn talon
    public double getAbsoluteTurnPosition() {
        double temp = 180 - encoderInvert * (turnEncoder.getAbsolutePosition() - encoderOffset) * turnConstants.degreesPerRotation;

        while(temp <= 0) {
            temp += 360;
        }
        while(temp > 360) {
            temp -= 360;
        }

        return temp;
    }

    //Metres/second velocity of the drive talon
    public double getDriveVelocity() {
        return driveRelative.getVelocity() * driveConstants.metresPerRotation / 60d;
    }

    //Set x and y position of the motor in meters. 
    public void resetPosition(double[] xy) {
        currPos[0] = xy[0];
        currPos[1] = xy[1];
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

    //Returns position of module
    public double[] getModulePosition() {
        return currPos;
    }

    //Realign the position of the wheels after figuring out the position of the center
    public void alignPosition(double[] center, double reducedAngle, double[] initial) {
        //Find out angle and distance from origin of initial
        //Polar coordinates
        double distance = Math.sqrt(Math.pow(initial[0], 2) + Math.pow(initial[1], 2));
        double angle = Math.atan2(initial[1], initial[0]) + Math.PI * 2 - Math.PI / 2;

        //Convert to degrees
        angle *= 180 / Math.PI;

        //Adjust angle 
        angle += reducedAngle;

        //Check for wrapping
        while (angle >= 360) {
            angle -= 360;
        }

        //Reconvert back to radians
        angle *= Math.PI / 180;

        //Reconvert back to cartesian coordinates
        double adjustedX = -Math.sin(angle) * distance;
        double adjustedY = Math.sin(angle) * distance;

        //Add robot values and set the new positions
        currPos[0] = center[0] + adjustedX;
        currPos[1] = center[1] + adjustedY;
    }
}
