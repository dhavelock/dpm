package ca.mcgill.ecse211.Navigator;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class Navigator extends Thread {
  
  private static final double SQUARE_LENGTH = 30.48;  // Length of single square on floor
  private static final int FORWARD_SPEED = 200;       // Regular forward speed
  private static final int ROTATE_SPEED = 150;        // Rotate speed
  private static final double CM_ERR = 1.0;           // Range to make it to target
  private static final int TRAVEL_SLEEP = 100;        // Time for this thread to sleep
  private static final int DIST_THRESHOLD = 12;       // Closest distance to obstacle
  private static final double DIST_AVOID = 37;        // Distance to travel around obstacle
  private static final double REVERSE_DIST = 5;       // Distance to reverse when obstacle is encountered
    
  private double x;             // x coordinate
  private double y;             // y coordinate
  private double theta;         // angle with respect to North (y axis)
  private boolean isAvoiding;   // true if obstacles involved. False otherwise
  
  private NavigatorObstacle obstacleAvoidance;    
  private Odometer odometer;
  
  /**
   * Constructor for No obstacle avoidance
   * @param odo
   * @throws OdometerExceptions
   */
  public Navigator(Odometer odo) throws OdometerExceptions {
    double[] odoData = odo.getXYT();
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
    this.odometer = Odometer.getOdometer();
    this.isAvoiding = false;
    this.obstacleAvoidance = null;

  }
  
  /**
   * Constructed for when obstacle avoidance is needed
   * @param odo
   * @param obstacleAvoidance
   * @throws OdometerExceptions
   */
  public Navigator(Odometer odo, NavigatorObstacle obstacleAvoidance) throws OdometerExceptions {
    double[] odoData = odo.getXYT();
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
    this.odometer = Odometer.getOdometer();
    this.isAvoiding = true;
    this.obstacleAvoidance = obstacleAvoidance;
  }
  
  /**
   * Sets private data members 
   * @param odoData
   */
  private void setOdoData(double[] odoData) {
    this.x = odoData[0];
    this.y = odoData[1];
    this.theta = odoData[2];
  }
  
  /**
   * Rotates robot to newTheta, taking into account the current value of theta
   * @param newTheta
   */
  private void turnTo (double newTheta) {
    
    // set motor speed for rotation
    Lab3.leftMotor.setSpeed(ROTATE_SPEED);
    Lab3.rightMotor.setSpeed(ROTATE_SPEED);
 
    double angleToRotate = newTheta - theta;

    if (angleToRotate < 0) {
      angleToRotate = angleToRotate + 360;
    }
    
    System.out.println("angle to rotate" + angleToRotate);
    
    // ensure minimal angle is always made
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
  
  /**
   * calculates the angle to (X, Y) assuming an initial heading of 0 degrees
   * @param X
   * @param Y
   * @return angle between 0 and 360 degrees
   */
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
  
  /**
   * Reverses robot and maneuvers around an obstacle at a 45 degree angle to the left
   */
  private void avoidLeft() {
    // Reverse
    Lab3.leftMotor.setSpeed(FORWARD_SPEED);
    Lab3.rightMotor.setSpeed(FORWARD_SPEED);
    
    Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -1 * REVERSE_DIST), true);
    Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -1 * REVERSE_DIST), false);
    
    // Turn Left
    Lab3.leftMotor.setSpeed(ROTATE_SPEED);
    Lab3.rightMotor.setSpeed(ROTATE_SPEED);

    Lab3.leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), true);
    Lab3.rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), false);
    
    // Avoid
    Lab3.leftMotor.setSpeed(FORWARD_SPEED);
    Lab3.rightMotor.setSpeed(FORWARD_SPEED);
    
    Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), true);
    Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), false);
  }
  
  /**
   * Reverses robot and maneuvers around an obstacle at a 45 degree angle to the right
   */
  private void avoidRight() {
    // Reverse
    Lab3.leftMotor.setSpeed(FORWARD_SPEED);
    Lab3.rightMotor.setSpeed(FORWARD_SPEED);
    
    Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -1 * REVERSE_DIST), true);
    Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, -1 * REVERSE_DIST), false);
    
    // Turn Right
    Lab3.leftMotor.setSpeed(ROTATE_SPEED);
    Lab3.rightMotor.setSpeed(ROTATE_SPEED);

    Lab3.leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), true);
    Lab3.rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 45.0), false);
    
    // Avoid
    Lab3.leftMotor.setSpeed(FORWARD_SPEED);
    Lab3.rightMotor.setSpeed(FORWARD_SPEED);
    
    Lab3.leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), true);
    Lab3.rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 1.41 * DIST_AVOID), false);
  }
  
  /**
   * Checks if the robot's x and y is within CM_ERR centimetres of X and Y
   * @param X
   * @param Y
   * @return false if robot reaches destination. true otherwise
   */
  private boolean isTravelling(double X, double Y) {
    return !(Math.abs(X - x) < CM_ERR && Math.abs(Y - y) < CM_ERR);
  }
  
  /**
   * Given X and Y, this method will navigate the robot to (X, Y)
   * @param X
   * @param Y
   */
  public void travelTo(double X, double Y) {
    
    // convert coordinates to usable values by odometer
    X = X*SQUARE_LENGTH;
    Y = Y*SQUARE_LENGTH;
    
    // calculate heading and point robot 
    double newHeading = calcNewHeading(X, Y);
    turnTo(newHeading);
    
    System.out.println("Travel to: (" + X + ", " + Y + ")");
    System.out.println("current  : (" + x + ", " + y + ")");
    
    // move forward robot until it reaches the point (X, Y)
    while (isTravelling(X, Y)) {
      double[] odoData = odometer.getXYT();
      setOdoData(odoData);
      
      // move forward
      Lab3.leftMotor.setSpeed(FORWARD_SPEED);
      Lab3.rightMotor.setSpeed(FORWARD_SPEED);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.forward();
      
      // put thread to sleep
      try {
        Thread.sleep(TRAVEL_SLEEP);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      // obstacle avoidance, only if isAvoiding = true
      int distance = 0;
      if (isAvoiding) {
          distance = obstacleAvoidance.getDistance();
          System.out.println("dist: " + distance + " | " + x + ", " + y + ", " + theta);
      }
      if (isAvoiding && distance <= DIST_THRESHOLD) {
        
        boolean turnLeft =    (y >= 1.8*SQUARE_LENGTH && theta >= 255 && theta <= 285)
                           || (x >= 1.8*SQUARE_LENGTH && (theta >= 345 || theta <= 15))
                           || (y <= 0.2*SQUARE_LENGTH && theta >= 75 && theta <= 105)
                           || (x <= 0.2*SQUARE_LENGTH && theta >= 165 && theta <= 195);

        if (turnLeft) {
          avoidLeft();
        } else {
          avoidRight();
        }

        turnTo(calcNewHeading(X, Y));
      }
      
    }
    Lab3.leftMotor.setSpeed(0);
    Lab3.rightMotor.setSpeed(0);
  }
  
  public void run() {
    for (int[] wayPoint : Lab3.WAYPOINTS) {
      travelTo(wayPoint[0], wayPoint[1]);
    }
  }
}
