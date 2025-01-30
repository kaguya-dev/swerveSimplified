package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveUtils.SwerveWheel;

public class Robot extends TimedRobot {
  private Joystick j1 = new Joystick(0);

  public SparkMax motorLeftFrontDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackDirection = new SparkMax(Constants.MOTOR_LEFT_ANGULAR_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackDirection = new SparkMax(Constants.MOTOR_RIGHT_ANGULAR_BACK, MotorType.kBrushless);

  public SparkMax motorLeftFrontSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_FRONT, MotorType.kBrushless);
  public SparkMax motorLeftBackSpeed = new SparkMax(Constants.MOTOR_LEFT_DRIVER_BACK, MotorType.kBrushless);
  public SparkMax motorRightFrontSpeed = new SparkMax(Constants.MOTOR_RIGHT_DRIVER_FRONT, MotorType.kBrushless);
  public SparkMax motorRightBackSpeed = new SparkMax(Constants.MOTOR_RIGHT_DRIVER_BACK, MotorType.kBrushless);

  public CANcoder encoder1 = new CANcoder(Constants.CANCODER_FRONT_LEFT);
  public CANcoder encoder2 = new CANcoder(Constants.CANCODER_BACK_LEFT);
  public CANcoder encoder3 = new CANcoder(Constants.CANCODER_FRONT_RIGHT);
  public CANcoder encoder4 = new CANcoder(Constants.CANCODER_BACK_RIGHT);

  private final PIDController pids = new PIDController(Constants.KP_Swerve_ANGLE,
      Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);

  SwerveWheel frontLeftSwerveWheel = new SwerveWheel(motorLeftFrontDirection,
      motorLeftFrontSpeed, encoder1, "1", pids);

  SwerveWheel backLeftSwerveWheel = new SwerveWheel(motorLeftBackDirection,
      motorLeftBackSpeed, encoder2, "2", pids);

  SwerveWheel frontRightSwerveWheel = new SwerveWheel(motorRightFrontDirection,
      motorRightFrontSpeed, encoder3, "3", pids);

  SwerveWheel backRightSwerveWheel = new SwerveWheel(motorRightBackDirection,
      motorRightBackSpeed, encoder4, "4", pids);

  private double finalpose;
  private boolean a;
  private double xToAngle, yToAngle, turnSpeed;

  @Override
  public void robotInit() {
    pids.setTolerance(0.01);
    pids.setIZone(0.1);
  }

  @Override
  public void teleopPeriodic() {
    a = j1.getRawButton(1);
    turnSpeed = j1.getRawAxis(2);
    finalpose = j1.getPOV();

    if (finalpose == 0)
      finalpose = 360;

    SmartDashboard.putNumber("finalpose", finalpose);

    if(Math.abs(turnSpeed) <= 0.1) translade();
    else turnIn(turnSpeed);
  }

  public double getCANcoderDegrees(double pose) {

    double degrees = ((pose + 1) % 1) * 360;

    if (degrees < 0) {
      degrees += 360;
    }
    return degrees;
  }

  public void translade() {

    SmartDashboard.putNumber("finalpose", finalpose / 360);

    SwerveWheel[] wheels = {
      frontLeftSwerveWheel,backLeftSwerveWheel,frontRightSwerveWheel,backRightSwerveWheel
    };

    xToAngle = -j1.getRawAxis(0);
    yToAngle = j1.getRawAxis(1);

    double angle = Units.radiansToRotations(Math.atan2(xToAngle, yToAngle)+ Math.PI);
    if(Math.abs(xToAngle) <= 0.04 && Math.abs(yToAngle) <= 0.04) angle = -1;

    SmartDashboard.putNumber("Analog1 angle", angle);

    for (int i = 0; i < wheels.length; i++) {

      double actualPose = wheels[i].getAbsoluteValue();

      if (angle != -1) {
        wheels[i].setDirection(angle);
      } else {

        wheels[i].stopDirection();
      }

      wheels[i].setSpeed(j1.getRawAxis(4));

      SmartDashboard.putNumber("actualpose" + (i + 1), actualPose);
    }
  }

  public void turnIn(double power){
    SwerveWheel[] wheels = {
      frontLeftSwerveWheel,backLeftSwerveWheel,frontRightSwerveWheel,backRightSwerveWheel
    };

    
    wheels[0].setDirection(Units.degreesToRotations(45));
    wheels[1].setDirection(Units.degreesToRotations(135));
    wheels[2].setDirection(Units.degreesToRotations(225));
    wheels[3].setDirection(Units.degreesToRotations(315));

    for (int i = 0; i < wheels.length; i++) {
      wheels[i].setSpeed(power);
       
    }
  }

}