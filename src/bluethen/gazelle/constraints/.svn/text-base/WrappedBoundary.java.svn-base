package bluethen.gazelle.constraints;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import bluethen.gazelle.Body;
import bluethen.gazelle.PhysicsMediator;

/*
 * Should keep track of all copies of poly
 * poly will be copied along each edge opposite of the edges it overlaps
 * the copies will be Locked to poly
 * once it completely goes over the edge, either the copy will replace the poly and poly is deleted
 *   or the copy is deleted
 */
public class WrappedBoundary implements Constraint {
	Rigid shape;
	float minX;
	float minY;
	float maxX;
	float maxY;
	
	// Map "MinX" "MaxX" "MinY" "MaxY" "MaxX MaxY" "MinX MinY" "MinX MaxY" "MaxX MinY" to Polygon copies
	Map<String, Rigid> shapeCopies;
	
	public WrappedBoundary(Rigid shape, float minX, float minY, float maxX, float maxY) {
		this.shape = shape;
		this.minX = minX;
		this.minY = minY;
		this.maxX = maxX;
		this.maxY = maxY;
		shapeCopies = new HashMap<String, Rigid>();
	}
	public void solveConstraint(PhysicsMediator physics, float width, float height) {
		// check to see if all bodies in polygon are overlapping edge
		// if none are, do nothing
		// if they are, completely move body to opposite side of screen
		// if only some are, create a "linked" body + or - one width/height along the axis
		
		boolean overlapMaxX = false;
		boolean allOverlapMaxX = true;
		
		boolean overlapMinX = false;
		boolean allOverlapMinX = true;
		
		boolean overlapMaxY = false;
		boolean allOverlapMaxY = true;
		
		boolean overlapMinY = false;
		boolean allOverlapMinY = true;
		for (Body b : shape.getBodies()) {
			if (b.getX() > (maxX-shape.getBoundingRadius())) // -width/2
				overlapMaxX = true;
			if (b.getX() < maxX)
				allOverlapMaxX = false;
			
			if (b.getX() < (minX+shape.getBoundingRadius())) // +width/2
				overlapMinX = true;
			if (b.getX() > minX)
				allOverlapMinX = false;
			
			if (b.getY() > (maxY-shape.getBoundingRadius())) // -height/2
				overlapMaxY = true;
			if (b.getY() < maxY)
				allOverlapMaxY = false;
			
			if (b.getY() < (minY+shape.getBoundingRadius())) // +height/2
				overlapMinY = true;
			if (b.getY() > minY)
				allOverlapMinY = false;
		}
		if (!overlapMinX && !overlapMaxX && !overlapMinY && !overlapMaxY)
			return;
		
//		System.out.println("1 " + overlapMinX + " " + overlapMaxX + " " + overlapMinY + " " + overlapMaxY);
//		System.out.println("2 " + allOverlapMinX + " " + allOverlapMaxX + " " + allOverlapMinY + " " + allOverlapMaxY);
		
		// edge cases
		overlap(physics, "MaxX", overlapMaxX, allOverlapMaxX, -(maxX-minX), 0);
		overlap(physics, "MinX", overlapMinX, allOverlapMinX, maxX-minX, 0);
		overlap(physics, "MaxY", overlapMaxY, allOverlapMaxY, 0, -(maxY-minY));
		overlap(physics, "MinY", overlapMinY, allOverlapMinY, 0, maxY-minY);
		
		// corner cases
		overlap(physics, "MaxX MaxY", overlapMaxX && overlapMaxY, allOverlapMaxX && allOverlapMaxY,
				-(maxX-minX), -(maxY-minY));
		overlap(physics, "MinX MaxY", overlapMinX && overlapMaxY, allOverlapMinX && allOverlapMaxY,
				maxX-minX, -(maxY-minY));
		overlap(physics, "MaxX MinY", overlapMaxX && overlapMinY, allOverlapMaxX && allOverlapMinY, 
				-(maxX-minX), maxY-minY);
		overlap(physics, "MinX MinY", overlapMinX && overlapMinY, allOverlapMinX && allOverlapMinY, 
				maxX-minX, maxY-minY);
		
	}
	void overlap (PhysicsMediator physics, String name, boolean overlap, boolean overlapAll, 
			float xOff, float yOff) {
		if (overlap) {
			if (overlapAll) {
				// all overlaps edge
				// remove copy and move parent into its position
				if (shapeCopies.containsKey(name)) {
					// destroy the MaxX copy
					shapeCopies.get(name).destroySelf(physics);
					shapeCopies.remove(name);
				}
				for (Body b : shape.getBodies()) {
					float vX = b.getVelX();
					float vY = b.getVelY();
					b.setX(b.getX() + xOff);
					b.setY(b.getY() + yOff);
					b.setLastX(b.getX() - vX);
					b.setLastY(b.getY() - vY);
				}
				shape.solveConstraint(physics.getGrid(), false, true);
			}
			else { // only a part of the poly overlaps the edge
				// create a clone of the body and lock it
				if (!shapeCopies.containsKey(name)) {
					
					Rigid pClone = shape.clone(physics);
					
					// translate pClone to opposite side of boundary
					// match velocities
					Body[] bodies = shape.getBodies();
					Body[] bClones = pClone.getBodies();
 					for (int i = 0; i < bodies.length; i++) {
 						bClones[i].setX(bodies[i].getX() + xOff);	
 						bClones[i].setY(bodies[i].getY() + yOff);
 						bClones[i].setLastX(bClones[i].getX() - bodies[i].getVelX());
 						bClones[i].setLastY(bClones[i].getY() - bodies[i].getVelY());
					}
					// apply translation locks
					for (int i = 0; i < bodies.length; i++) {
						Lock lock = new Lock(bodies[i], bClones[i], xOff, yOff);
						physics.addConstraint(lock);
					}
					// keep track of clone
					shapeCopies.put(name, pClone);
				}
			}
		}
		else { // We're not overlapping the edge
			if (shapeCopies.containsKey(name)) {
				shapeCopies.get(name).destroySelf(physics);
				shapeCopies.remove(name);
			}
		}
		
	}
}
