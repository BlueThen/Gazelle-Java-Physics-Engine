package bluethen.gazelle.constraints;

import java.util.List;

import bluethen.gazelle.Body;

public class Boundary implements Constraint {
	Body body;
	float minX;
	float minY;
	float maxX;
	float maxY;
	float damping;
	float slidingFriction = 0.1f;

	public Boundary(Body body, float minX, float minY, float maxX, float maxY) {
		this.body = body;
		this.minX = minX;
		this.minY = minY;
		this.maxX = maxX;
		this.maxY = maxY;
		this.damping = 0.95f;
	}

	public void solveConstraint(List<Body> bodyPool, boolean preserveVelocity) {
		float vx = 0, vy = 0;
		if (preserveVelocity) {
			vx = body.getVelX();
			vy = body.getVelY();
		}

		if (body.getX() < minX) {
			// Correct body's location
			body.setX(minX);

			// Apply sliding friction
			// float vyNorm = vy / body.getVelocityMagnitude();
			// body.setLastY(body.getLastY() + vx * vyNorm * slidingFriction);

			float mag = (float) Math.sqrt(vx * vx + vy * vy);
			if (mag != 0 && vx < 0) {
				body.setLastY(body.getY() - vy * (1f - (vx / mag) * slidingFriction));
			}
			// Reset vY
			// vy = (body.getLastY() - body.getY());

			// Preserve x-velocity
			if (preserveVelocity)
				body.setLastX(body.getX() + vx);
			vx = body.getVelX();
		}
		if (body.getX() > maxX) {
			body.setX(maxX);

			// float vyNorm = vy / body.getVelocityMagnitude();
			// body.setLastY(body.getLastY() + vx * vyNorm * slidingFriction);

			float mag = (float) Math.sqrt(vx * vx + vy * vy);
			if (mag != 0 && body.getVelX() > 0) {
				body.setLastY(body.getY() - vy * (1f - (vx / mag) * slidingFriction));
				// System.out.println(vy + " vs " + body.getVelY());
			}

			if (preserveVelocity)
				body.setLastX(body.getX() + vx);
			vx = body.getVelX();

		}
		if (body.getY() < minY) {
			body.setY(minY);

			// float vxNorm = vx / body.getVelocityMagnitude();
			// body.setLastX(body.getLastX() + vy * vxNorm * slidingFriction);

			float mag = (float) Math.sqrt(vx * vx + (vy + body.getAccY()) * (vy + body.getAccY()));
			if (mag != 0 && vy + body.getAccY() > 0) {
				body.setLastX(body.getX() - vx * (1f - ((vy + body.getAccY()) / mag) * slidingFriction));
			}

			if (preserveVelocity)
				body.setLastY(body.getY() + vy);
			vy = body.getVelY();
		}
		if (body.getY() > maxY) {
			// System.out.println("normal force: " + (maxY - body.getY()) +
			// " vs " + vy);
			body.setY(maxY);

			float mag = (float) Math.sqrt(vx * vx + (vy + body.getAccY())
					* (vy + body.getAccY()));
			if (mag != 0 && vy + body.getAccY() > 0) {
				body.setLastX(body.getX() - vx * (1f - ((vy + body.getAccY()) / mag) * slidingFriction));
			}

			if (preserveVelocity)
				body.setLastY(body.getY() + vy);
			vy = body.getVelY();
		}
		body.solveLocks();
	}

}
