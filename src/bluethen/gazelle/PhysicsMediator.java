package bluethen.gazelle;

import java.util.ArrayList;
import java.util.List;

import bluethen.gazelle.constraints.Ball;
import bluethen.gazelle.constraints.Constraint;
import bluethen.gazelle.constraints.MouseLocked;

/* 
 * Physics Mediator
 * Brings all of the managers together to perform physics
 * Should contain only list of bodies
 */

public class PhysicsMediator {
	TimestepCounter timestep;
	ConstraintManager constraints;
	List<Body> bodyPool;
	List<Ball> ballPool;
	
	boolean gravity = true;
	
	public int width, height;
	
	public PhysicsMediator(int width, int height) {
		timestep = new TimestepCounter(15, 3);
		constraints = new ConstraintManager(width, height, 1);
		bodyPool = new ArrayList<Body>();
		ballPool = new ArrayList<Ball>();
		
		System.out.println("Gazelle loaded 4");
		
		this.width = width;
		this.height = height;
	}

	public ConstraintManager getConstraintManager() {
		return constraints;
	}

	public void update(int elapsedTime) {
		for (timestep.update(elapsedTime); timestep.hasNext(); timestep.decrement()) {
			VerletMotion.accelerate(bodyPool, timestep.getTimestep());
			constraints.solveConstraints(this, bodyPool, false);
			VerletMotion.inertia(bodyPool, ballPool, timestep.getTimestep());
			
			if (gravity)
				applyGravity(bodyPool);

			constraints.solveConstraints(this, bodyPool, true);
		}
	}
	public void registerBall (Ball b) {
		ballPool.add(b);
	}
	public void registerBody (Body b) {
		bodyPool.add(b);
	}
	public void removeBody (Body b) {
		bodyPool.remove(b);
	}
	public Body getBody (int i) {
		return bodyPool.get(i);
	}
	public int bodyCount () {
		return bodyPool.size();
	}
	public void setGravity (boolean toggle) {
		gravity = toggle;
	}
	public boolean getGravity () {
		return gravity;
	}
	
	public void addConstraint(Constraint constraint) {
		if (constraint instanceof Ball)
			ballPool.add((Ball)constraint);
		constraints.addConstraint(constraint);
	}
	public void removeConstraint(Constraint constraint) {
		if (constraint instanceof Ball)
			ballPool.remove((Ball)constraint);
		constraints.removeConstraint(constraint);
	}

	void applyGravity(List<Body> bodies) {
		for (Body body : bodies) {
			body.setAccY(body.getAccY() + 392f);
		}
	}

	void addVel(List<Body> bodyPool, float radius, int x, int y, int lastX, int lastY, 
			float velX, float velY) {
		float radiusSquared = radius * radius;
		for (Body body : bodyPool) {
			if (distPointToSegmentSquared(x, y, lastX, lastY, body.getX(),
					body.getY()) < radiusSquared) {
				body.setLastX(body.getLastX() - velX);
				body.setLastY(body.getLastY() - velY);
			}
		}
	}

	public Grid getGrid () {
		return constraints.getGrid();
	}
	
	// Credit to:
	// http://www.codeguru.com/forum/showpost.php?p=1913101&postcount=16
	float distPointToSegmentSquared(float lineX1, float lineY1, float lineX2, float lineY2, 
			float pointX, float pointY) {
		float vx = lineX1 - pointX;
		float vy = lineY1 - pointY;
		float ux = lineX2 - lineX1;
		float uy = lineY2 - lineY1;

		float len = ux * ux + uy * uy;
		float det = (-vx * ux) + (-vy * uy);
		if ((det < 0) || (det > len)) {
			ux = lineX2 - pointX;
			uy = lineY2 - pointY;
			return Math.min(vx * vx + vy * vy, ux * ux + uy * uy);
		}

		det = ux * vy - uy * vx;
		return (det * det) / len;
	}
}
