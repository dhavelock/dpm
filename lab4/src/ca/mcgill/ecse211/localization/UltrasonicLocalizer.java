package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.Navigator.DistanceController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class UltrasonicLocalizer extends Thread {
  
  private static final int ROTATE_SPEED = 90;  // Rotate speed
  private static final int SLEEP_TIME = 20;     // Time for this thread to sleep
  private static final int TOP_EDGE = 100;      // above this threshold is considered looking away from the wall
  private static final int BOTTOM_EDGE = 20;      // below this threshold is considered looking at the wall

  private Odometer odometer;                    // odometer
  private DistanceController distanceController; // US Poller
  private double angleEdge1;
  private double angleEdge2;
  private boolean atWall;                            // true = looking at wall, false = looking away
  
  int mode;                                     // 0 = falling edge, 1 = rising edge
  
  public UltrasonicLocalizer(int mode, DistanceController dc) throws OdometerExceptions {
    this.mode = mode;
    this.odometer = Odometer.getOdometer();
    this.distanceController = dc;
    this.angleEdge1 = -1;
    this.angleEdge2 = -1;
    this.atWall = false;
  }
  
  public void fallingEdge() {
    double theta = odometer.getTheta();
    
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
    Main.leftMotor.forward();
    Main.rightMotor.backward();
    
    atWall = false;
    
    while (angleEdge1 == -1 || angleEdge2 == -1) {
      theta = odometer.getTheta();
      int dist = distanceController.getDistance();
      
      System.out.println(dist + " " + theta);
      
      if (dist > TOP_EDGE) {
        atWall = false;
      } 
      
      if (dist < BOTTOM_EDGE && !atWall) {
        if (angleEdge1 == -1) {
          angleEdge1 = theta;
          
          Main.leftMotor.setSpeed(ROTATE_SPEED);
          Main.rightMotor.setSpeed(ROTATE_SPEED);
          Main.leftMotor.backward();
          Main.rightMotor.forward();
          
        } else if (angleEdge2 == -1) {
          angleEdge2 = theta;
          break;
        }     
        
        atWall = true;
      }
      
      try {
        Thread.sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    double angleToRotate = (angleEdge1 + angleEdge2) / 2 + 135 - theta;
    
    rotate(angleToRotate);
    
    System.out.println("angle1: " + angleEdge1);
    System.out.println("angle2: " + angleEdge2);
    
  }
  
  public void risingEdge() {
double theta = odometer.getTheta();
    
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
    Main.leftMotor.forward();
    Main.rightMotor.backward();
    
    atWall = true;
    
    while (angleEdge1 == -1 || angleEdge2 == -1) {
      theta = odometer.getTheta();
      int dist = distanceController.getDistance();
      
      System.out.println(dist + " " + theta);
      
      if (dist < BOTTOM_EDGE) {
        atWall = true;
      } 
      
      if (dist > TOP_EDGE && atWall) {
        if (angleEdge1 == -1) {
          angleEdge1 = theta;
          
          Main.leftMotor.setSpeed(ROTATE_SPEED);
          Main.rightMotor.setSpeed(ROTATE_SPEED);
          Main.leftMotor.backward();
          Main.rightMotor.forward();
          
        } else if (angleEdge2 == -1) {
          angleEdge2 = theta;
          break;
        }     
        
        atWall = false;
      }
      
      try {
        Thread.sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    double angleToRotate = (angleEdge1 + angleEdge2) / 2 - 45 + (360 - theta);
    
    rotate(angleToRotate);
    
    System.out.println("angle1: " + angleEdge1);
    System.out.println("angle2: " + angleEdge2);
  }
  
  private void rotate(double angle) {
    // set motor speed for rotation
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
 
    
    System.out.println("angle to rotate " + angle);
    
    // ensure minimal angle is always made
    Main.leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, angle), true);
    Main.rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, angle), false);
    
    Main.leftMotor.setSpeed(0);
    Main.rightMotor.setSpeed(0);
  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  public void run() {
    if (mode == 0) {
      fallingEdge();
    }
    if (mode == 1) {
      risingEdge();
    }
    if (mode == 2) {
      rotate(360);
    }
  }
  
}
