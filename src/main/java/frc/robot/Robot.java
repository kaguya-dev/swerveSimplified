package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

  public Pigeon2 gyro = new Pigeon2(15);

  private final PIDController pids = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);

  SwerveWheel frontLeftSwerveWheel = new SwerveWheel(motorLeftFrontDirection,motorLeftFrontSpeed, encoder1, "1", pids);
  SwerveWheel backLeftSwerveWheel = new SwerveWheel(motorLeftBackDirection, motorLeftBackSpeed, encoder2, "2", pids);
  SwerveWheel frontRightSwerveWheel = new SwerveWheel(motorRightFrontDirection, motorRightFrontSpeed, encoder3, "3", pids);
  SwerveWheel backRightSwerveWheel = new SwerveWheel(motorRightBackDirection, motorRightBackSpeed, encoder4, "4", pids);

  SwerveWheel[] wheels = { frontLeftSwerveWheel, backLeftSwerveWheel, backRightSwerveWheel, frontRightSwerveWheel };

  private double finalpose;
  private double xToAngle, yToAngle, turnSpeed, l2, r2;
  private double magTranslade;
  private double yaw,roll,pitch;
  double angleTranslade;

  @Override
  public void robotInit() {
    pids.setTolerance(0.01);
    pids.setIZone(0.1);
  }

  @Override
  public void teleopPeriodic() {
    turnSpeed = j1.getRawAxis(4);
    finalpose = j1.getPOV();

    yaw = gyro.getYaw().getValueAsDouble();
    pitch = gyro.getPitch().getValueAsDouble();
    roll = gyro.getRoll().getValueAsDouble();


    xToAngle = -j1.getRawAxis(0);
    yToAngle = j1.getRawAxis(1);

    l2 = j1.getRawAxis(2);
    r2 = j1.getRawAxis(3);

    magTranslade = Math.sqrt(Math.pow(xToAngle,2) + Math.pow(yToAngle, 2));

    angleTranslade = Units.radiansToRotations(Math.atan2(xToAngle, yToAngle) + Math.PI);
    if (Math.abs(xToAngle) <= 0.05 && Math.abs(yToAngle) <= 0.05)
      angleTranslade = -1;

    if (finalpose == 0)
      finalpose = 360;

    SmartDashboard.putNumber("finalpose", finalpose);
    SmartDashboard.putNumberArray("Angles", new double[]{yaw, pitch, roll});

    if ((Math.abs(xToAngle) >= 0.05) || (Math.abs(yToAngle) >= 0.05))
      translade();
    else if (j1.getRawButton(4))
      turnIn();
    else{
      motorsOff();
    }
    speedOn();

  }

  public void speedOn(){
    for (int i = 0; i < wheels.length; i++)
      wheels[i].setSpeed(r2 - l2);
  }

  public void motorsOff(){

    for(int i = 0; i < wheels.length; i++){
      wheels[i].stopDirection();
      wheels[i].setSpeed(0);
    }
  }
  public void translade() {

    SmartDashboard.putNumber("finalpose", finalpose / 360);
    SmartDashboard.putNumber("Analog1 angleTranslade", angleTranslade);

    for (int i = 0; i < wheels.length; i++) {

      double actualPose = wheels[i].getAbsoluteValue();

      if (angleTranslade != -1) {
        wheels[i].setDirection(angleTranslade);
      } else {

        wheels[i].stopDirection();
      }

      SmartDashboard.putNumber("actualpose" + (i + 1), actualPose);
    }
  }

  public void turnIn() {
    
    wheels[0].setDirection(Units.degreesToRotations(45));
    wheels[1].setDirection(Units.degreesToRotations(135));
    wheels[2].setDirection(Units.degreesToRotations(225));
    wheels[3].setDirection(Units.degreesToRotations(315));

    for (int i = 0; i < wheels.length; i++) {
      wheels[i].setSpeed(power);
       
    }
    wheels[0].setDirection(0.125);
    wheels[1].setDirection(0.885);
    wheels[2].setDirection(0.635);
    wheels[3].setDirection(0.375);

  }

  public void translateTurn(double direction, double translatePower, double turnPower) {

    double turnAngle = turnPower * 45.0;

    // if the left front wheel is in the front
    if (wheels[0].closestAngle(direction, 135.0) >= 90.0) {
      wheels[0].setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      wheels[0].setDirection(direction - turnAngle);
    }
    // if the left back wheel is in the front
    if (wheels[1].closestAngle(direction, 225.0) > 90.0) {
      wheels[1].setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      wheels[1].setDirection(direction - turnAngle);
    }
    // if the right front wheel is in the front
    if (wheels[3].closestAngle(direction, 45.0) > 90.0) {
      wheels[3].setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      wheels[3].setDirection(direction - turnAngle);
    }
    // if the right back wheel is in the front
    if (wheels[2].closestAngle(direction, 315.0) >= 90.0) {
      wheels[2].setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      wheels[2].setDirection(direction - turnAngle);
    }

    wheels[0].setSpeed(translatePower);
    wheels[1].setSpeed(translatePower);
    wheels[2].setSpeed(translatePower);
    wheels[3].setSpeed(translatePower);

    
  }
}
