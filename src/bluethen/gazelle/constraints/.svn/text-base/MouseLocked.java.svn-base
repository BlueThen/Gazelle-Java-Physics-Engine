package bluethen.gazelle.constraints;

import java.util.List;

import bluethen.gazelle.Body;
import bluethen.gazelle.ConstraintManager;
import bluethen.gazelle.PhysicsMediator;

public class MouseLocked implements Constraint {
	static float mouseX, mouseY;
	static float pmouseX, pmouseY;

	//static Body body;
	static boolean lockNext = false;
	static boolean locked;
	static float offsetX;
	static float offsetY;

	static Rigid rigidBody;
	// The mouse will be distance-locked from each of rigidBody's vertices
	static float[] distances;

	static public void pinBody (PhysicsMediator physics) {
		Body pin = new Body(mouseX, mouseY);
		physics.registerBody(pin);
		physics.addConstraint(new Pinned(pin));
		if (locked && rigidBody != null) {
			Body[] bodiesToLockTo = rigidBody.getBodies();
			for (int i = 0; i < bodiesToLockTo.length; i++) {
				Body b = bodiesToLockTo[i];
				physics.addConstraint(new Link(b, pin, distances[i], 1f));
			}
//			rigidBody.correctBody();
			locked = false;
			lockNext = false;
		}
	}
	static public void solveConstraint() {
		if (locked) {
			if (rigidBody != null) {
				Body[] bodiesToLockTo = rigidBody.getBodies();
				for (int i = 0; i < bodiesToLockTo.length; i++) {
					Body b = bodiesToLockTo[i];

					// Find x and y difference of both bodies
					float deltaX = mouseX - b.getX();
					float deltaY = mouseY - b.getY();

					// Get distance
					float distance = (float) Math.sqrt(deltaX * deltaX + deltaY * deltaY);

					// Find the difference and normalize it
					float difference = (distances[i] - distance) / distance;
					if (Math.abs(difference) > 0.01) {
						// Correct the body positions
						b.setX(b.getX() - deltaX * difference);
						b.setY(b.getY() - deltaY * difference);
					}

				}
				rigidBody.correctBody();
			}
			if (lockNext) {
				locked = false;
				lockNext = false;
			}
		} 
	}

	static public void mouseMoved(int oldx, int oldy, int newx, int newy) {
		mouseX = newx;
		mouseY = newy;

		pmouseX = oldx;
		pmouseY = oldy;
	}

	static public void mousePressed(ConstraintManager constraints, int button, int x, int y) {
		if (!locked) {
			// Find if the mouse is inside any rigid body
			for (Rigid rb : constraints.getRigidBodies()) {
				if (rb.contains(x, y)) {

					Body[] bodiesToLockTo = rb.getBodies();

					distances = new float[bodiesToLockTo.length];
					for (int i = 0; i < bodiesToLockTo.length; i++) {
						distances[i] = (float) Math.sqrt(Math.pow(x - bodiesToLockTo[i].getX(), 2) + Math.pow(y - bodiesToLockTo[i].getY(), 2));
					}

					rigidBody = rb;
					locked = true;
					return;
				}
			}

		}

	}
	
	static public void mouseReleased(int arg0, int arg1, int arg2) {
		if (locked) {
			// lockNext tells us that the mouse is no longer pressing
			// it allows the constraint one more solve before letting go
			// this is necessary to transfer mouse's velocity over
			lockNext = true;
		}
	}

	public static boolean isLocked () {
		return locked;
	}
}
