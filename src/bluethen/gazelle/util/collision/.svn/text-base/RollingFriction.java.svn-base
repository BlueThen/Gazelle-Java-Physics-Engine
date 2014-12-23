package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Ball;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.util.PMath;

/*
 * Provides methods for performing sliding friction
 */
public class RollingFriction {
	public static boolean enabled = true;
	public static void apply (Ball ballA, Ball ballB,
			float collisionX, float collisionY,
			float collisionNormalX, float collisionNormalY, 
			float rollingFriction) {
		if (enabled) {
//			Body body = ball.getBody();
			
			// separation axis
			float rX = -collisionNormalY;
			float rY = collisionNormalX;

			// normal
			float uX = collisionNormalX;
			float uY = collisionNormalY;
			
			float ballAVX = ballA.getBody().getVelX();
			float ballAVY = ballA.getBody().getVelY();
			float ballAVelMag = ballA.getBody().getVelocityMagnitude();
			float ballAVA = ballA.getAngleVel();
			float ballAVAlinear = ballAVA * ballA.getRadius();
			
			float ballBVX = ballB.getBody().getVelX();
			float ballBVY = ballB.getBody().getVelY();
			float ballBVelMag = ballB.getBody().getVelocityMagnitude();
			float ballBVA = ballB.getAngleVel();
			
			float ballBVAlinear = ballBVA * ballB.getRadius();
			
			float aVdotN = PMath.dotP(ballAVX, ballAVY, uX, uY);
		    float bVdotN = PMath.dotP(ballBVX, ballBVY, -uX, -uY);
		    

			float addToAngleVelA = 0, addToAngleVelB = 0;

			float ballAVatEdge = PMath.dotP(ballAVX, ballAVY, rX, rY);
			float ballBVatEdge = PMath.dotP(ballBVX, ballBVY, rX, rY);

			if (bVdotN > 0 && aVdotN > 0) {
				addToAngleVelA = ballAVatEdge - ballBVatEdge; // + ballBVAlinear; //* (bVdotN / ballBVelMag) * rollingFriction;
				addToAngleVelB = ballBVatEdge - ballAVatEdge; // + ballAVAlinear; // * (aVdotN / ballAVelMag) * rollingFriction;
			
//			float temp = addToAngleVelA;
//			addToAngleVelA -= addToAngleVelB;
//			addToAngleVelB -= temp;
		
				ballA.setLastAngle(ballA.getAngle() - addToAngleVelA/ballA.getRadius());//(ballAVA + addToAngleVelA/ballA.getRadius()));
				ballB.setLastAngle(ballB.getAngle() - addToAngleVelB/ballB.getRadius());//(ballBVA + addToAngleVelB/ballB.getRadius()));
			}
		}
	}
}
