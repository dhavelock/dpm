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
  
  private static final double LIGHT_OFFSET = 11.2;   // measured distance from light sensor to center of robot
  private static final double LIGHT_THRESHOLD = 0.45; // light value that signals a line is crossed, determined experimentally 
  private static final int ROTATE_SPEED = 100;
  private static final int SLEEP_TIME = 50;     // Time for this thread to sleep
  private static final int MOVE_DIST = 10;
  private static final double CM_ERR = 0.75;

  
  private static Port lsPort = LocalEV3.get().getPort("S3"); 
  private SampleProvider sampleProvider;
  private SensorModes lightSensor;
  private float[] lsData;
  
  private double[] angles;
  private int x;
  private int y;
  
  private Odometer odometer;
  
  public LightLocalizer() throws OdometerExceptions {
    
    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
    this.sampleProvider = lightSensor.getMode("Red");
    this.lsData = new float[lightSensor.sampleSize()];
    angles = new double[4];
  }
  
  public void localize() {
    double dy = LIGHT_OFFSET * Math.abs(Math.cos(Math.toRadians(angles[2] - angles[0])/2));
    double dx = LIGHT_OFFSET * Math.abs(Math.cos(Math.toRadians(angles[3] - angles[1])/2));
    
    odometer.setX(-dx);
    odometer.setY(-dy);
    
    System.out.println(odometer.getX() + " " + odometer.getY() + " " + odometer.getTheta());
  }
  
public void travelTo(double X, double Y) {
    
    // calculate heading and point robot 
    rotate(360 - odometer.getTheta() + 45);
    
    System.out.println("Travel to: (" + X + ", " + Y + ")");
    System.out.println("current  : (" + x + ", " + y + ")");
    
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
  
private boolean isTravelling(double X, double Y) {
  return !(Math.abs(X - odometer.getX()) < CM_ERR && Math.abs(Y - odometer.getY()) < CM_ERR);
}

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
    gatherPoints();
    localize();
    travelTo(0, 0);
  }

}
