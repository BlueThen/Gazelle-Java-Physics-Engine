package bluethen.gazelle;

import java.util.List;

import bluethen.gazelle.constraints.Ball;

/*
 * Verlet Motion Manager
 * Uses verlet integration to integrate motion.
 */
public class VerletMotion {
	static void accelerate(List<Body> bodies, float timestep) {
		for (Body body : bodies) {
			// x' = x + a * t^2
			float nextX = body.getX() + body.getAccX() * timestep * timestep;
			float nextY = body.getY() + body.getAccY() * timestep * timestep;

			body.setX(nextX);
			body.setY(nextY);

			// Reset acceleration
			body.setAccX(0);
			body.setAccY(0);
		}
	}

	static void inertia(List<Body> bodies, List<Ball> balls, float timestep) {
		for (Body body : bodies) {
			// v = x' - x
			float velX = body.getVelX();
			float velY = body.getVelY();

			// x' = x + v
			float nextX = body.getX() + velX;
			float nextY = body.getY() + velY;

			// Reset velocity
			body.setLastX(body.getX());
			body.setLastY(body.getY());

			body.setX(nextX);
			body.setY(nextY);
		}
		for (Ball ball : balls) {
			float angleVel = ball.getAngleVel();
			
			float nextAngle = ball.getAngle() + angleVel;
			
			ball.setLastAngle(ball.getAngle());
			
			ball.setAngle(nextAngle);
		}
	}
}
