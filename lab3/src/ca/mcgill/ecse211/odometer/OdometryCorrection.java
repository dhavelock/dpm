/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final int MAX_DOUBLE_LINE = 1200;		// max time between consecutive line hits
  private static final long CORRECTION_PERIOD = 10;		// correction period
  private static final double TILE_LENGTH = 30.48;		// length of a single square on competition board
  private static final double LIGHT_THRESHOLD = 0.10;	// light value that signals a line is crossed, determined experimentally 
  private static final double START_X = 0.0;			// starting X coordinate of robot
  private static final double START_Y = 0.0;			// starting Y coordinate of robot
  private static final double LIGHT_OFFSET = 2.7;		// measured distance from light sensor to center of robot
  														//    which is directly between the wheels
  private Odometer odometer;

  private static Port lsPort = LocalEV3.get().getPort("S1"); 
  private SampleProvider sampleProvider;
  private SensorModes lightSensor;
  private float[] lsData;
  private int numXLines;
  private int numYLines;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.lightSensor = new EV3ColorSensor(lsPort);
	this.sampleProvider = lightSensor.getMode("Red");
	this.lsData = new float[lightSensor.sampleSize()];
	this.numXLines = 0;
	this.numYLines = 0;

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    float colourIntensity;
    double theta;
    
    odometer.setXYT(START_X, START_Y, 0);

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position
      // TODO Update odometer with new calculated (and more accurate) vales
      
      sampleProvider.fetchSample(lsData, 0);
      colourIntensity = lsData[0];
      double[] odoData = odometer.getXYT();
      theta = odoData[2];
      boolean hitLine = false;
                  
      if (colourIntensity <= LIGHT_THRESHOLD) {
    	  hitLine = true;
    	  Sound.beep();
    	  
    	  if (theta <= 10 || theta >= 350) { // facing North
    		  odometer.setY(numYLines*TILE_LENGTH - LIGHT_OFFSET);
    		  numYLines++;
    	  } else if (theta >= 80 && theta <= 100) { // facing East
    		  odometer.setX(numXLines*TILE_LENGTH - LIGHT_OFFSET);
    		  numXLines++;
    	  } else if (theta >= 170 && theta <= 190) { // facing South
    		  numYLines--;
    		  odometer.setY(numYLines*TILE_LENGTH + LIGHT_OFFSET);
    	  } else if (theta >= 260 && theta <= 280) { // facing West
    		  numXLines--;
    		  odometer.setX(numXLines*TILE_LENGTH + LIGHT_OFFSET);
    	  }
    	  
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (hitLine) { // add extra delay if a line was just passed
  		try {
			Thread.sleep(MAX_DOUBLE_LINE);
		} catch (InterruptedException e) {
			// do nothing
		}
  	  }
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));          
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
