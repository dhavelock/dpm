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
  private static final double LIGHT_THRESHOLD = 0.35; // light value that signals a line is crossed, determined experimentally 
  private static final int ROTATE_SPEED = 100;
  private static final int SLEEP_TIME = 50;     // Time for this thread to sleep

  
  private static Port lsPort = LocalEV3.get().getPort("S3"); 
  private SampleProvider sampleProvider;
  private SensorModes lightSensor;
  private float[] lsData;
  
  private Odometer odometer;
  
  public LightLocalizer() throws OdometerExceptions {
    
    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
    this.sampleProvider = lightSensor.getMode("Red");
    this.lsData = new float[lightSensor.sampleSize()];
  }
  
  public void gatherPoints() {
    float colourIntensity;
    
    sampleProvider.fetchSample(lsData, 0);
    colourIntensity = lsData[0];
    double theta = odometer.getTheta();
    
    Main.leftMotor.setSpeed(ROTATE_SPEED);
    Main.rightMotor.setSpeed(ROTATE_SPEED);
    Main.leftMotor.forward();
    Main.rightMotor.backward();
    
    while(true) {
      sampleProvider.fetchSample(lsData, 0);
      colourIntensity = lsData[0];
      
      System.out.println(colourIntensity);
      
      if (colourIntensity <= LIGHT_THRESHOLD) {
        Sound.beep();
      }
      
      Main.leftMotor.setSpeed(ROTATE_SPEED);
      Main.rightMotor.setSpeed(ROTATE_SPEED);
      Main.leftMotor.forward();
      Main.rightMotor.backward();
      
      try {
        Thread.sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
  }
  
  public void run() {
    gatherPoints();
  }

}
