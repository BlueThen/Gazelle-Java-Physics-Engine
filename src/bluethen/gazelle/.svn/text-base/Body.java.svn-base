package bluethen.gazelle;

import java.util.ArrayList;
import java.util.List;

import org.newdawn.slick.geom.Rectangle;
import org.newdawn.slick.geom.Shape;

import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Link;
import bluethen.gazelle.constraints.Lock;

/* 
 * Point
 * Contains all spatial and physical data
 */
public class Body {
	// Physics
	private float x, y;
	private float lastX, lastY; // Used for inertia
	// private float lastCorrectedX, lastCorrectedY; // Used for
	                                                 // collision checking
	private float accX, accY;
	private float mass;
	
	// The outside boundary of the body
//	private Shape shape;
	private boolean collidable;

	// For if the ball is pinned/mouseLocked/etc
	private boolean locked;

	// Links: distance constraints from other bodies
	private List<Link> links;
	// Locks: offset constraints from other bodies
	private List<Lock> locks;
	// Edges
	private List<Edge> edges;

	public Body() {
		this(0, 0);
	}

	public Body(float x, float y) {
		this.x = x;
		this.y = y;
		lastX = x;
		lastY = y;
	
		mass = 1;
		accX = 0;
		accY = 0;

//		shape = new Rectangle(x, y, 20, 20);

		links = new ArrayList<Link>();
		locks = new ArrayList<Lock>();
		edges = new ArrayList<Edge>();
		
		collidable = false;
	}

	public void addLink(Link link) {
		links.add(link);
	}
	public boolean linkedTo (Body b) {
		for (Link l : links)
			if (l.getBody1() == b || l.getBody2() == b)
				return true;
		return false;
	}
	public void removeLink(Link link) {
		links.remove(link);
	}
	public List<Link> getLinks() {
		return links;
	}
	
	public void addEdge(Edge edge) {
		edges.add(edge);
	}
	public boolean edgedTo (Body b) {
		for (Edge e : edges)
			if (e.getBody1() == b || e.getBody2() == b)
				return true;
		return false;
	}
	public void removeEdge(Edge edge) {
		edges.remove(edge);
	}
	public List<Edge> getEdges() {
		return edges;
	}
	
	public void addLock(Lock lock) {
		locks.add(lock);
	}

	public void removeLock(Lock lock) {
		locks.remove(lock);
	}
	public List<Lock> getLocks() {
		return locks;
	}
	public void solveLocks() {
		for (Lock l : locks)
			l.solveConstraints(this);
	}
	
	public boolean isCollidable() {
		return collidable;
	}

	public void setCollidable(boolean collidable) {
		this.collidable = collidable;
	}

	public boolean isLocked() {
		return locked;
	}

	public void setLocked(boolean locked) {
		this.locked = locked;
	}

	public float getX() {
		return x;
	}

	public float getY() {
		return y;
	}

	public float getLastX() {
		return lastX;
	}

	public float getLastY() {
		return lastY;
	}

	public float getAccX() {
		return accX;
	}

	public float getAccY() {
		return accY;
	}

	public float getMass() {
		return mass;
	}

	public float getVelocityMagnitude() {
		float vx = x - lastX;
		float vy = y - lastY;
		return (float) Math.sqrt(vx * vx + vy * vy);
	}

	public float getVelX() {
		return x - lastX;
	}

	public float getVelY() {
		return y - lastY;
	}

	public void setX(float x) {
		this.x = x;
	}

	public void setY(float y) {
		this.y = y;
	}

	public void setLastX(float lastX) {
		this.lastX = lastX;
	}

	public void setLastY(float lastY) {
		this.lastY = lastY;
	}

	public void setAccX(float accX) {
		this.accX = accX;
	}

	public void setAccY(float accY) {
		this.accY = accY;
	}

	public void setMass(float mass) {
		this.mass = mass;
	}

//	public void setShape(Shape shape) {
//		this.shape = shape;
//	}
//
//	public Shape getShape() {
//		shape.setCenterX(x);
//		shape.setCenterY(y);
//		return shape;
//	}
	
	public void addVelX(float amt) {
		lastX -= amt;
	}
	public void addVelY(float amt) {
		lastY -= amt;
	}
	
	public Body clone (PhysicsMediator physics) {
		Body clone = new Body(x,y);
		
		clone.lastX = lastX;
		clone.lastY = lastY;
		
		clone.accX = accX;
		clone.accY = accY;
		
		clone.mass = mass;
		
		clone.collidable = collidable;
		
		clone.locked = locked;
		
		// don't bother copying links/edges

		physics.registerBody(clone);
		
		return clone;
	}
}
