package ca.mcgill.ecse211.Navigator;

import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.lab3.Lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator implements Runnable {
  
  private static final double SQUARE_LENGTH = 30.48;
  private static final int FORWARD_SPEED = 220;
  private static final int ROTATE_SPEED = 150;
    
  private double x;
  private double y;
  private double theta;
  
  private int currWayPointX;
  private int currWayPointY;
  private int currWayPoint;
  
  private int lastWayPointX;
  private int lastWayPointY;
    
  private Odometer odometer;
  
  public Navigator(Odometer odo, int[] wayPoints) throws OdometerExceptions {
                                              // manipulation methods    
    double[] odoData = odo.getXYT();
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
    
    this.currWayPointX = wayPoints[0];
    this.currWayPointY = wayPoints[1];
    this.currWayPoint = 0;
    
    this.lastWayPointX = 0;
    this.lastWayPointY = 0;
    
    this.odometer = Odometer.getOdometer();

  }
  
  private void setOdoData(double[] odoData) {
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
  }
  
  private void turnTo (double newTheta) {
    
    Lab3.leftMotor.setSpeed(ROTATE_SPEED);
    Lab3.rightMotor.setSpeed(ROTATE_SPEED);
 
    double angleToRotate = newTheta - theta;

    if (angleToRotate < 0) {
      angleToRotate = angleToRotate + 360;
    }
    
    System.out.println("angle to rotate" + angleToRotate);
    
    
    
    if (angleToRotate < 180) {
      Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angleToRotate), true);
      Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angleToRotate), false);
    } else {
      Lab3.leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 360-angleToRotate), true);
      Lab3.rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 360-angleToRotate), false);
    }

  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  public int[] getCurrWayPoint() {
    int[] wayPoint = new int[2];
    
    wayPoint[0] = this.currWayPointX;
    wayPoint[1] = this.currWayPointY;
    
    return wayPoint;
  }
  
  public void run() {
    
    odometer.setXYT(0, 0, 0);
        
    while (true) {
      
      double[] odoData = odometer.getXYT();
      setOdoData(odoData);
      
      System.out.println(x + ", " + y + ", " + theta + " | " + currWayPointX + ", " + currWayPointY);
      
      if (Math.abs(currWayPointX*SQUARE_LENGTH - x) < 2 && Math.abs(currWayPointY*SQUARE_LENGTH - y) < 2) {
        Lab3.leftMotor.stop();
        Lab3.rightMotor.stop();
        currWayPoint = currWayPoint + 2;
      } 
            
      if (currWayPoint == Lab3.wayPoints.length) {
        System.exit(0);
      } else if (Lab3.leftMotor.isMoving() == false && Lab3.rightMotor.isMoving() == false) {
        currWayPointX = Lab3.wayPoints[currWayPoint];
        currWayPointY = Lab3.wayPoints[currWayPoint+1];
        
        if (currWayPoint >= 2) {
          lastWayPointX = Lab3.wayPoints[currWayPoint-2];
          lastWayPointY = Lab3.wayPoints[currWayPoint-1];
        }
        
        System.out.println("waypoint = " + currWayPoint);
        System.out.println(" currX= " + currWayPointX + " currY= " + currWayPointY);
        System.out.println(" lastX= " + lastWayPointX + " lastY= " + lastWayPointY);
        
        double newHeading;
        int dx = currWayPointX - lastWayPointX;
        int dy = currWayPointY - lastWayPointY;
        
        if (dy == 0) {
          System.out.println("Ys are equal");
          if (dx < 0) {
            newHeading = 270;
          } else {
            newHeading = 90;
          }
        } else {              
          newHeading = Math.abs(Math.toDegrees(Math.atan((double) dx / (double) dy)));
          
          System.out.println("dx " + dx + " dy " + dy);
          System.out.println("** heading " + newHeading);
          
          if (dx >= 0 && dy < 0) {
            newHeading = 180 - newHeading;
          }
          
          if (dx <= 0 && dy > 0) {
            newHeading = 360 - newHeading;
          }
          
          if (dx < 0 && dy < 0) {
            newHeading = 180 + newHeading;
          }
        }
        
        System.out.println("new heading" + newHeading);
        turnTo(newHeading);
        
        Lab3.leftMotor.setSpeed(FORWARD_SPEED);
        Lab3.rightMotor.setSpeed(FORWARD_SPEED);
        
        double travelDist = Math.sqrt(Math.pow(currWayPointX*SQUARE_LENGTH - x, 2) + Math.pow(currWayPointY*SQUARE_LENGTH - y, 2));

        Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, travelDist), true);
        Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, travelDist), false);
        
      }
      try {
        Thread.sleep(10);          
      } catch (InterruptedException e) {
        // there is nothing to be done here
      }
    }
    
  }
}
