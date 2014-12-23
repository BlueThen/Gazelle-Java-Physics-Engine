package bluethen.gazelle.util.collision;

import bluethen.gazelle.Body;
import bluethen.gazelle.constraints.Ball;
import bluethen.gazelle.constraints.Circle;

public class CircleToCircle {
	public static boolean resolve (Circle circleA, Circle circleB, float damping, boolean preserveVelocity) {
		if (circleA != circleB) {
			Body bodyA = circleA.getBody();
			Body bodyB = circleB.getBody();
			
			// get radii
			float radiusA = circleA.getRadius();
			float radiusB = circleB.getRadius();

			// check if they intersect
			float deltaX = bodyA.getX() - bodyB.getX();
			float deltaY = bodyA.getY() - bodyB.getY();
			float deltaSquared = (float) (Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
			if (deltaSquared <= Math.pow(radiusA + radiusB, 2)) {

				// Previous velocities
				float v1x = 0, v1y = 0, v2x = 0, v2y = 0;
				if (preserveVelocity) {
					v1x = bodyA.getX() - bodyA.getLastX();
					v1y = bodyA.getY() - bodyA.getLastY();
					v2x = bodyB.getX() - bodyB.getLastX();
					v2y = bodyB.getY() - bodyB.getLastY();
				}
				
				float distance = (float) Math.sqrt(deltaSquared);

				if (distance == 0) {
					distance = radiusA + radiusB - 1;
					deltaX = radiusA + radiusB;
					deltaY = 0;
				}
				float difference = ((radiusA + radiusB) - distance) / distance;
				float mtdX = deltaX * difference;
				float mtdY = deltaY * difference;
				
				float inverseMass1 = 1f / bodyA.getMass();
				float inverseMass2 = 1f / bodyB.getMass();

				// Correction is the fraction of the translation
				// distance each point should get
				// It depends on their mass differences and the
				// stiffness of the link
				float correction1 = (inverseMass1 / (inverseMass1 + inverseMass2));
				float correction2 = (inverseMass2 / (inverseMass1 + inverseMass2));
				if (!bodyA.isLocked() && !bodyB.isLocked()) {
					// Compute how much to move each body based on mass
					

					// Push-pull based on mass
					bodyA.setX(bodyA.getX() + mtdX * correction1);
					bodyA.setY(bodyA.getY() + mtdY * correction1);

					bodyB.setX(bodyB.getX() - mtdX * correction2);
					bodyB.setY(bodyB.getY() - mtdY * correction2);
					

				} else if (bodyA.isLocked() && !bodyB.isLocked()) {
					bodyB.setX(bodyB.getX() - mtdX);
					bodyB.setY(bodyB.getY() - mtdY);
				} else if (!bodyA.isLocked() && bodyB.isLocked()) {
					bodyA.setX(bodyA.getX() + mtdX);
					bodyA.setY(bodyA.getY() + mtdY);
				}
				if (circleA instanceof Ball && circleB instanceof Ball)
					RollingFriction.apply((Ball) circleA, (Ball) circleB,
							bodyA.getX() - 0.5f * mtdX * correction1, bodyA.getY() - 0.5f * mtdY * correction1,
							deltaX / distance, deltaY / distance, 
							1f);
				// Preserve Velocity
				if (preserveVelocity) {
					PreserveVelocity.apply(bodyA, bodyB,
							v1x, v1y, v2x, v2y, 
							deltaX/distance, deltaY/distance,
							damping);
				}
				return true;
			}
		}
		return false;
	}
}
