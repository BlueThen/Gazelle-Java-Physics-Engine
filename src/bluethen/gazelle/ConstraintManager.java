package bluethen.gazelle;

import java.util.ArrayList;
import java.util.List;
import bluethen.gazelle.constraints.*;

public class ConstraintManager {
	List<Constraint> constraints;

	List<Pinned> pinConstraints;
	List<Boundary> boundaryConstraints;
	List<WrappedBoundary> wrappedBoundaryConstraints;
	
	List<Rigid> rigidConstraints;

	List<Link> linkConstraints;
	List<Edge> edgeConstraints;
	
	List<Lock> lockConstraints;

	Grid rigidBodyGrid;

	int repeat = 1;

	ConstraintManager(int width, int height) {
		this(width, height, 1);
	}

	ConstraintManager(int width, int height, int repeat) {
		this.repeat = repeat;
		
		constraints = new ArrayList<Constraint>();
		pinConstraints = new ArrayList<Pinned>();
		boundaryConstraints = new ArrayList<Boundary>();
		wrappedBoundaryConstraints = new ArrayList<WrappedBoundary>();
		rigidConstraints = new ArrayList<Rigid>();
		linkConstraints = new ArrayList<Link>();
		edgeConstraints = new ArrayList<Edge>();
		lockConstraints = new ArrayList<Lock>();
		
		rigidBodyGrid = new Grid(width, height);
	}

	public List<Link> getLinks() {
		return linkConstraints;
	}

	void solveConstraints(PhysicsMediator physics, List<Body> bodyPool, boolean preserveVelocity) {
		for (int i = 0; i < repeat; i++) {
			rigidBodyGrid.populate(rigidConstraints);

			// Constraint priority: Pinned > Boundary > MouseLocked > Polygon >
			// Edge > Link > Circle

			for (Lock lock : lockConstraints)
				lock.solveConstraint();
			for (Link link : linkConstraints)
				link.solveConstraint();
			for (Edge edge : edgeConstraints)
				edge.solveConstraint(bodyPool, preserveVelocity);
			
			for (Rigid rigidBody : rigidConstraints)
				rigidBody.solveConstraint(rigidBodyGrid, preserveVelocity, false);
			MouseLocked.solveConstraint();
			
			for (Boundary boundary : boundaryConstraints)
				boundary.solveConstraint(bodyPool, preserveVelocity);
			for (WrappedBoundary wrapped : wrappedBoundaryConstraints)
				wrapped.solveConstraint(physics, physics.width, physics.height);
			
			for (Pinned pinned : pinConstraints)
				pinned.solveConstraint();
			
//			for (Constraint constraint : constraints)
//				constraint.solveConstraint();
		}
		
	}

	void addConstraint(Constraint constraint) {

		if (constraint instanceof Pinned)
			pinConstraints.add((Pinned) constraint);
		else if (constraint instanceof Boundary)
			boundaryConstraints.add((Boundary) constraint);
		else if (constraint instanceof WrappedBoundary)
			wrappedBoundaryConstraints.add((WrappedBoundary) constraint);
		else if (constraint instanceof Rigid) {
			rigidConstraints.add((Rigid) constraint);
			// make sure grid can fit it
			((Rigid) constraint).calculateBoundingRadius();
			rigidBodyGrid.check(((Rigid) constraint).getBoundingRadius());	
		} 
		else if (constraint instanceof Edge)
			edgeConstraints.add((Edge) constraint);
		else if (constraint instanceof Link)
			linkConstraints.add((Link) constraint);
		else if (constraint instanceof Lock)
			lockConstraints.add((Lock) constraint);

		else // unheard of constraints
			constraints.add(constraint);

	}
	void removeConstraint(Constraint constraint) {
		if (constraint instanceof Pinned)
			pinConstraints.remove((Pinned) constraint);
		else if (constraint instanceof Boundary)
			boundaryConstraints.remove((Boundary) constraint);
		else if (constraint instanceof WrappedBoundary)
			wrappedBoundaryConstraints.remove((WrappedBoundary) constraint);
		else if (constraint instanceof Rigid) {
			rigidConstraints.remove((Rigid) constraint);
			// make sure grid can fit it
			rigidBodyGrid.removeFromCell(((Rigid) constraint), ((Rigid) constraint).getCell());	
		} 
		else if (constraint instanceof Edge)
			edgeConstraints.remove((Edge) constraint);
		else if (constraint instanceof Link)
			linkConstraints.remove((Link) constraint);
		else if (constraint instanceof Lock)
			lockConstraints.remove((Lock) constraint);

		else // unheard of constraints
			constraints.remove(constraint);
		
	}
	public List<Edge> getEdges() {
		return edgeConstraints;
	}

	public List<Rigid> getRigidBodies() {
		return rigidConstraints;
	}
	
	public Grid getGrid () { 
		return rigidBodyGrid;
	}
	// public List<Circle> getCircles() {
	// return circleConstraints;
	// }
}
