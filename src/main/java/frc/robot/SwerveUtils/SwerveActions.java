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

  public static void turnOut(SwerveWheel[] frontWheels, SwerveWheel[] backWheels, double angleTranslade, double diffPower){

    for (int i = 0; i < frontWheels.length; i++) {

      double actualPoseF = frontWheels[i].getAbsoluteValue();
      double actualPoseB = backWheels[i].getAbsoluteValue();

      if (angleTranslade != -1) {
        frontWheels[i].setDirection(angleTranslade);
        backWheels[i].setDirection(angleTranslade + (diffPower/2));
      } else {
        frontWheels[i].stopDirection();
        backWheels[i].stopDirection();
      }

      SmartDashboard.putNumber("frontal actualpose" + (i + 1), actualPoseF);
      SmartDashboard.putNumber("back actualpose" + (i + 1), actualPoseB);
    }
  }

  public static void turnIn(SwerveWheel[] wheels) {

    wheels[0].setDirection(0.125);
    wheels[1].setDirection(0.875);
    wheels[2].setDirection(0.625);
    wheels[3].setDirection(0.375);
  }
}
