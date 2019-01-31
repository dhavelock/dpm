package ca.mcgill.ecse211.Navigator;

import ca.mcgill.ecse211.lab3.Lab3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    Lab3.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    Lab3.rightMotor.setSpeed(MOTOR_SPEED);
    Lab3.leftMotor.forward();
    Lab3.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    
    int distError = this.distance - this.bandCenter;
    int diff = calcGain(distError);
    
    if (Math.abs(distError) <= this.bandWidth) { // within desired bandwidth
      Lab3.leftMotor.setSpeed(MOTOR_SPEED);
      Lab3.rightMotor.setSpeed(MOTOR_SPEED);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.forward();
        
    } else if (this.distance <= 15) { // extremely too close
      Lab3.leftMotor.setSpeed(MOTOR_SPEED);
      Lab3.rightMotor.setSpeed(MOTOR_SPEED);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.backward();
        
    } else if (distError > 0) { // too far
      Lab3.leftMotor.setSpeed(MOTOR_SPEED - diff);
      Lab3.rightMotor.setSpeed(MOTOR_SPEED + diff);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.forward();
        
    } else if (distError < 0) { // too close
      Lab3.leftMotor.setSpeed(MOTOR_SPEED + diff);
      Lab3.rightMotor.setSpeed(MOTOR_SPEED - diff);
      Lab3.leftMotor.forward();
      Lab3.rightMotor.forward();
    }
    
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  private int calcGain(int diff) {
	  // different proportions for each scenario
	  double propTooClose = 4.2;
	  double propTooFar = 2.0;
	  
	  int maxCorrection;
	  int correction;
	  
	  if (diff < 0) { // too close
		  diff = Math.abs(diff);
		  maxCorrection = 60;
		  correction = (int) (propTooClose * (double) diff);
	  } else { // too far
		  maxCorrection = 43;
		  correction = (int) (propTooFar * (double) diff);
	  }
	  
	  if (correction >= maxCorrection) { // set a cap on the maximum value
		  correction = maxCorrection;
	  }
	  return correction;
  }

}
