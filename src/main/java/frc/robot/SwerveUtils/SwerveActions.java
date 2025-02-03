package frc.robot.SwerveUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveActions {

  public static void speedOn(SwerveWheel[] wheels, double power) {
    for (int i = 0; i < wheels.length; i++)
      wheels[i].setSpeed(power);
  }

  public static void motorsOff(SwerveWheel[] wheels) {

    for (int i = 0; i < wheels.length; i++) {
      wheels[i].stopDirection();
      wheels[i].setSpeed(0);
    }
  }

  public static void translade(SwerveWheel[] wheels, double angleTranslade) {

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

  public static void turnOut(SwerveWheel[] frontWheels, SwerveWheel[] backWheels, double angleTranslade,
      double diffPower) {
    for (int i = 0; i < frontWheels.length; i++) {
      //String fWheelID = frontWheels[i].WheelID;
      //String bWheelID = backWheels[i].WheelID;
      if (angleTranslade != -1) {
        diffPower = (diffPower / 8);
        double actualPoseF = rounder((angleTranslade - (diffPower)*(i*1.25)));
        double actualPoseB = rounder((angleTranslade + (diffPower)*(i*1.25)));
        frontWheels[i].setDirection(actualPoseF);
        backWheels[i].setDirection(actualPoseB);

      } else {
        frontWheels[i].stopDirection();
        backWheels[i].stopDirection();
      }
    }
  }

  public static void turnIn(SwerveWheel[] wheels, double directionAxis) {
    //if (directionAxis > 0) {
      wheels[0].setDirection(0.125);
      wheels[1].setDirection(0.875);
      wheels[2].setDirection(0.625);
      wheels[3].setDirection(0.375);
    //} else if (directionAxis < 0) {
      //wheels[0].setDirection(0.625);
      //wheels[1].setDirection(0.375);
      //wheels[2].setDirection(0.125);
      //wheels[3].setDirection(0.875);
    //}
  }

  private static double rounder(double actualPose) {
    if (actualPose > 1)
      return actualPose -= 1;
    else if (actualPose < 1)
      return actualPose += 1;
    return actualPose;
  }
}
