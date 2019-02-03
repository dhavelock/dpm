package ca.mcgill.ecse211.Navigator;

import ca.mcgill.ecse211.lab3.Lab3;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class Navigator extends Thread {
  
  private static final double SQUARE_LENGTH = 30.48;
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 150;
  private static final double CM_ERR = 1.0;
  private static final int TRAVEL_SLEEP = 100;
  private static final int DIST_THRESHOLD = 15;
  private static final double DIST_AVOID = 34;
    
  private double x;
  private double y;
  private double theta;
  private boolean isAvoiding;
  
  private NavigatorObstacle obstacleAvoidance;    
  private Odometer odometer;
  
  public Navigator(Odometer odo, boolean isAvoiding) throws OdometerExceptions {
                                              // manipulation methods    
    double[] odoData = odo.getXYT();
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
    this.odometer = Odometer.getOdometer();
    this.isAvoiding = isAvoiding;
    this.obstacleAvoidance = null;

  }
  
  public Navigator(Odometer odo, NavigatorObstacle obstacleAvoidance, boolean isAvoiding) throws OdometerExceptions {
    // manipulation methods    
    double[] odoData = odo.getXYT();
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
    this.odometer = Odometer.getOdometer();
    this.isAvoiding = isAvoiding;
    this.obstacleAvoidance = obstacleAvoidance;
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
    
    Lab3.leftMotor.setSpeed(0);
    Lab3.rightMotor.setSpeed(0);

  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  private double calcNewHeading(double X, double Y) {
    double[] odoData = odometer.getXYT();
    setOdoData(odoData);
    
    double newHeading;
    double dx = X - x;
    double dy = Y - y;
    
    if (dy == 0) {
      if (dx < 0) {
        newHeading = 270;
      } else {
        newHeading = 90;
      }
    } else {              
      newHeading = Math.abs(Math.toDegrees(Math.atan((double) dx / (double) dy)));
      
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
    return newHeading;
  }
  
  public void travelTo(double X, double Y) {
    
    X = X*SQUARE_LENGTH;
    Y = Y*SQUARE_LENGTH;
        
    double[] odoData = odometer.getXYT();
    setOdoData(odoData);
    
    System.out.println(x + " " + y + " " + theta);

    double newHeading;
    double dx = X - x;
    double dy = Y - y;
    
    if (dy == 0) {
      if (dx < 0) {
        newHeading = 270;
      } else {
        newHeading = 90;
      }
    } else {              
      newHeading = Math.abs(Math.toDegrees(Math.atan((double) dx / (double) dy)));
      
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
    
    turnTo(newHeading);
    
    System.out.println("Travel to: (" + X + ", " + Y + ")");
    System.out.println("current  : (" + x + ", " + y + ")");
    
    while (isTravelling(X, Y)) {
      odoData = odometer.getXYT();
      setOdoData(odoData);
      
      Lab3.leftMotor.setSpeed(FORWARD_SPEED);
      Lab3.rightMotor.setSpeed(FORWARD_SPEED);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.forward();
      
      try {
        Thread.sleep(TRAVEL_SLEEP);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      // obstacle avoidance
      int distance = 0;
      if (isAvoiding) {
          distance = obstacleAvoidance.getDistance();
          System.out.println("dist: " + distance);
      }
      if (isAvoiding && distance <= DIST_THRESHOLD) {
        
        // Reverse
        Lab3.leftMotor.setSpeed(FORWARD_SPEED);
        Lab3.rightMotor.setSpeed(FORWARD_SPEED);
        
        Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -12), true);
        Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -12), false);
        
        // Turn
        Lab3.leftMotor.setSpeed(ROTATE_SPEED);
        Lab3.rightMotor.setSpeed(ROTATE_SPEED);

        Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), true);
        Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), false);
        
        // Avoid
        Lab3.leftMotor.setSpeed(FORWARD_SPEED);
        Lab3.rightMotor.setSpeed(FORWARD_SPEED);
        
        Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), true);
        Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), false);

        turnTo(calcNewHeading(X, Y));
      }
      
    }
    Lab3.leftMotor.setSpeed(0);
    Lab3.rightMotor.setSpeed(0);
  }
  
  private double[] getFutureCoords() {
    double[] futureLoc = new double[3];
    double thetaRad = Math.toRadians(theta);
    
    if (theta >= 0 && theta < 90) { // facing North
      futureLoc[0] = x + DIST_AVOID*Math.sin(thetaRad);
      futureLoc[1] = y + DIST_AVOID*Math.cos(thetaRad);
      
    } else if (theta >= 90 && theta < 180) { // facing East
      futureLoc[0] = x + DIST_AVOID*Math.abs(Math.sin(thetaRad));
      futureLoc[1] = y - DIST_AVOID*Math.abs(Math.cos(thetaRad));
    } else if (theta >= 180 && theta < 270) { // facing South
      futureLoc[0] = x - DIST_AVOID*Math.abs(Math.sin(thetaRad));
      futureLoc[1] = y - DIST_AVOID*Math.abs(Math.cos(thetaRad));
    } else if (theta >= 279 && theta < 360) { // facing West
      futureLoc[0] = x - DIST_AVOID*Math.abs(Math.sin(thetaRad));
      futureLoc[1] = y + DIST_AVOID*Math.abs(Math.cos(thetaRad));
    }
    
    return futureLoc;
  }
  
  private double absoluteDist(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
  }
  
  private boolean isTravelling(double X, double Y) {
    return !(Math.abs(X - x) < CM_ERR && Math.abs(Y - y) < CM_ERR);
  }
  
  public void run() {
    for (int[] wayPoint : Lab3.WAYPOINTS) {
      travelTo(wayPoint[0], wayPoint[1]);
    }
  }
}
