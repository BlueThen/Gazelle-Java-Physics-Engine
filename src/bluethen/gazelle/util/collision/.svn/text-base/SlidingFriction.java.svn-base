package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.util.PMath;

/*
 * Provides methods for performing sliding friction
 */
public class SlidingFriction {
	public static boolean enabled = true;
	public static void apply (Body body, float collisionNormalX, float collisionNormalY, float slidingFriction) {
		if (enabled) {
			// separation axis
			float rX = -collisionNormalY;
			float rY = collisionNormalX;
	
			// collision normal (pointed towards the body)
			float uX = collisionNormalX;
			float uY = collisionNormalY;
			
			float bodyVX = body.getVelX();
			float bodyVY = body.getVelY();
			float bodyVMag = body.getVelocityMagnitude();
			
			float vSep = PMath.dotP(bodyVX, bodyVY, rX, rY);
			float vNorm = PMath.dotP(bodyVX, bodyVY, uX, uY);
	
			if (vNorm > 0) {
				float newVSep = vSep * (1f - (vNorm / bodyVMag) * slidingFriction);
				float newVNorm = 0;
				if (PreserveVelocity.enabled)
					newVNorm = (float) Math.sqrt(bodyVMag * bodyVMag - newVSep * newVSep); // vNorm;
				else
					newVNorm = vNorm;
				// try to convert back to world space
				float newVX = newVSep * rX + newVNorm * uX;
				float newVY = newVSep * rY + newVNorm * uY;
	
				body.setLastX(body.getX() - newVX);
				body.setLastY(body.getY() - newVY);
			}
		}
	}
	public static void apply (Edge edge, float collisionNormalX, float collisionNormalY, float slidingFriction) {
		apply(edge.getBody1(), collisionNormalX, collisionNormalY, slidingFriction);
		apply(edge.getBody2(), collisionNormalX, collisionNormalY, slidingFriction);
	}
}
