package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread {
  
  private static final double LIGHT_OFFSET = 11.2;    // measured distance from light sensor to center of robot
  private static final double LIGHT_THRESHOLD = 0.45; // light value that signals a line is crossed, determined experimentally 
  private static final int ROTATE_SPEED = 100;        // Wheel speed 
  private static final int SLEEP_TIME = 50;           // Time for this thread to sleep
  private static final int MOVE_DIST = 10;            // Distance to move toward origin before light sensor routine
  private static final double CM_ERR = 0.75;          // Error threshold for navigating to origin

  
  private static Port lsPort = LocalEV3.get().getPort("S3");  // light sensor port
  private SampleProvider sampleProvider;                      // sample provider for light sensor
  private SensorModes lightSensor;                            // light sensor
  private float[] lsData;                                     // stores light sensor data
  
  private double[] angles;        // stores the angles of each line intersection
  
  private Odometer odometer;      // odometer instance
  
  /**
   * Class Constructor
   * @throws OdometerExceptions
   */
  public LightLocalizer() throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
    this.sampleProvider = lightSensor.getMode("Red");
    this.lsData = new float[lightSensor.sampleSize()];
    angles = new double[4];
  }
  
  /**
   * calculates x, y coordinates based off of the angles to the line intersection
   */
  public void localize() {
    double dy = LIGHT_OFFSET * Math.abs(Math.cos(Math.toRadians(angles[2] - angles[0])/2));
    double dx = LIGHT_OFFSET * Math.abs(Math.cos(Math.toRadians(angles[3] - angles[1])/2));
    
    odometer.setX(-dx);
    odometer.setY(-dy);
    
    System.out.println(odometer.getX() + " " + odometer.getY() + " " + odometer.getTheta());
  }
  
  /**
   * Navigates robot to the 0, 0 point on the grid
   */
  public void travelToOrigin() {
    
    double X = 0;
    double Y = 0;
    
    // point robot to origin
    rotate(360 - odometer.getTheta() + 45);
    
    System.out.println("Travel to: (" + X + ", " + Y + ")");
    
    // move forward robot until it reaches the point (X, Y)
    while (isTravelling(X, Y)) {
      
      // move forward
      Main.leftMotor.setSpeed(ROTATE_SPEED);
      Main.rightMotor.setSpeed(ROTATE_SPEED);
      Main.leftMotor.forward();
      Main.rightMotor.forward();
      
      System.out.println(odometer.getX() + " " + odometer.getY() + " " + odometer.getTheta());
      
      // put thread to sleep
      try {
        Thread.sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      
    }
    Main.leftMotor.setSpeed(0);
    Main.rightMotor.setSpeed(0);
    
    turnToZero();
  }
  
  /**
   * Determines whether the robot has reached the point X, Y
   * @param X
   * @param Y
   * @return true if robot is within error of target threshold, false otherwise
   */
  private boolean isTravelling(double X, double Y) {
    return !(Math.abs(X - odometer.getX()) < CM_ERR && Math.abs(Y - odometer.getY()) < CM_ERR);
  }

  /**
   * Rotates the robot such that the light sensor detects 4 lines
   * Stores the angle of each intersection in data member "angles"
   */
  public void gatherPoints() {
    float colourIntensity;
    
    odometer.setTheta(0);
    
    sampleProvider.fetchSample(lsData, 0);
    colourIntensity = lsData[0];
    double theta = odometer.getTheta();
 
    rotate(45);
    
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);

    Main.leftMotor.rotate(convertDistance(Main.WHEEL_RAD, MOVE_DIST), true);
    Main.rightMotor.rotate(convertDistance(Main.WHEEL_RAD, MOVE_DIST), false);
    
    int numLines = 0;
    
    while(numLines < 4) {
      Main.leftMotor.setSpeed(ROTATE_SPEED);
      Main.rightMotor.setSpeed(ROTATE_SPEED);
      Main.leftMotor.forward();
      Main.rightMotor.backward();
      
      sampleProvider.fetchSample(lsData, 0);
      colourIntensity = lsData[0];
      
      System.out.println(colourIntensity);
      
      theta = odometer.getTheta();
      if (colourIntensity <= LIGHT_THRESHOLD) {
        Sound.beep();
        
        angles[numLines] = theta;
        numLines++;
      }
      
      if (numLines == 4) {
        break;
      }
        
      try {
        Thread.sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    Main.leftMotor.setSpeed(0);
    Main.rightMotor.setSpeed(0);
    
    System.out.println("angles: ");
    for (double a : angles) {
      System.out.println(a);
    }
  }
  
  /**
   * Rotate robot until it the light sensor detects a black line
   * This aligns the robot perfectly with the North direction
   */
  private void turnToZero() {
    
    sampleProvider.fetchSample(lsData, 0);
    float colourIntensity = lsData[0];
    
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
    Main.leftMotor.backward();
    Main.rightMotor.forward();
    
    while (colourIntensity >= LIGHT_THRESHOLD) {
      sampleProvider.fetchSample(lsData, 0);
      colourIntensity = lsData[0];
    }
    
    Main.leftMotor.setSpeed(0);
    Main.rightMotor.setSpeed(0);
  }
  
  /**
   * Given an angle, rotate the robot clockwise that angle
   * @param angle (degrees)
   */
  private void rotate(double angle) {
    // set motor speed for rotation
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
     
    // rotate clockwise
    Main.leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, angle), true);
    Main.rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, angle), false);
    
    // stop robot
    Main.leftMotor.setSpeed(0);
    Main.rightMotor.setSpeed(0);
  }
  
  /**
   * utility method used by rotate
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * utility method used by rotate
   * @param radius
   * @param width
   * @param angle
   * @return
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  /**
   * Main run method.
   * 1) Gathers set of angles to each line detection
   * 2) calculates the current x, y based off angles
   * 3) navigates to the origin and redirects heading
   */
  public void run() {
    gatherPoints();
    localize();
    travelToOrigin();
  }

}
