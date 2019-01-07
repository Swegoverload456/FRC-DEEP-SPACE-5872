package frc.robot.Leds;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Led{

    Spark leds = new Spark(Constants.kBlinkinID);
    Timer timer = new Timer();

    public void off(){

        leds.set(0.99);

    }

    public void set(double color){

        leds.set(color);

    }

    public void blink(double color, int interval, int numberOfBlinks){

        //c = counter for toggle
        int c = 0;

        for(int i = 0; i < numberOfBlinks; i++){

            set(color);
            delay(interval);
            off();
            delay(interval);

        }    

    }

    public void delay(int ms){

        try {
			
			Thread.sleep(ms);
			
		}
		catch(Exception e1){
			
			e1.printStackTrace();
			
		}

    }

}