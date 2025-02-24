package frc.robot.SwerveUtils;

import edu.wpi.first.math.util.Units;

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

    for (int i = 0; i < wheels.length; i++) {

      if (angleTranslade != -1) {
        wheels[i].setDirection(angleTranslade);
      } else {
        wheels[i].stopDirection();
      }
    }
  }

  public static void turnOut(SwerveWheel[] frontWheels, SwerveWheel[] backWheels, double angleTranslade,
      double diffPower) {

    if (angleTranslade != -1) {
      diffPower = Units.degreesToRotations(diffPower * 45);
      double actualPoseF = (angleTranslade + (diffPower));
      double actualPoseB = (angleTranslade - (diffPower));
      frontWheels[0].setDirection(actualPoseF);
      frontWheels[1].setDirection(actualPoseF);
      backWheels[0].setDirection(actualPoseB);
      backWheels[1].setDirection(actualPoseB);

    } else {
      frontWheels[0].stopDirection();
      frontWheels[1].stopDirection();
      backWheels[0].stopDirection();
      backWheels[1].stopDirection();
    }

  }

  public static void transladeTurn(SwerveWheel[] frontWheels, SwerveWheel[] backWheels, double angleTranslade,
      double diffPower) {
    diffPower = diffPower / 8;

    // if the left front wheel is in the front
    if (frontWheels[0].closestAngle(angleTranslade, 0.375) >= 0.25) {
      frontWheels[0].setDirection(angleTranslade + diffPower);
    }
    // if it's in the back
    else {
      frontWheels[0].setDirection(angleTranslade - diffPower);
    }
    // if the left back wheel is in the front
    if (backWheels[0].closestAngle(angleTranslade, 0.625) > 0.25) {
      backWheels[0].setDirection(angleTranslade + diffPower);
    }
    // if it's in the back
    else {
      backWheels[0].setDirection(angleTranslade - diffPower);
    }

    // if the right front wheel is in the front
    if (frontWheels[1].closestAngle(angleTranslade, 0.125) > 0.25) {
      frontWheels[1].setDirection(angleTranslade + diffPower);
    }
    // if it's in the back
    else {
      frontWheels[1].setDirection(angleTranslade - diffPower);
    }

    // if the right back wheel is in the front
    if (backWheels[1].closestAngle(angleTranslade, 0.875) >= 0.25) {
      backWheels[1].setDirection(angleTranslade + diffPower);
    }
    // if it's in the back
    else {
      backWheels[1].setDirection(angleTranslade - diffPower);
    }

  }

  public static void turnIn(SwerveWheel[] wheels, double directionAxis) {

    wheels[0].setDirection(0.625);
    wheels[1].setDirection(0.875);
    wheels[2].setDirection(0.375);
    wheels[3].setDirection(0.125);

  }
}
