package bluethen.gazelle.util.tools;

import org.newdawn.slick.GameContainer;

import bluethen.gazelle.Body;
import bluethen.gazelle.PhysicsMediator;
import bluethen.gazelle.constraints.Boundary;
import bluethen.gazelle.constraints.Circle;
import bluethen.gazelle.constraints.Edge;
import bluethen.gazelle.constraints.Link;
import bluethen.gazelle.constraints.Polygon;
import bluethen.gazelle.constraints.WrappedBoundary;

public class ShapeBuilder {
	public static Polygon createBox(GameContainer app, PhysicsMediator physics, 
			float x, float y, float width) {
		return createBox(app, physics, x, y, width, width, false);
	}
	public static Polygon createBox(GameContainer app, PhysicsMediator physics, 
			float x, float y, float width, float height) {
		return createBox(app, physics, x, y, width, height, false);
	}
	public static Polygon createBox(GameContainer app, PhysicsMediator physics, 
			float x, float y, float width, float height, boolean wrappedEdge) {
		Polygon rb = new Polygon();

		Body a = new Body(x - width / 2, y - height / 2);
		Body b = new Body(x - width / 2, y + height / 2);
		Body c = new Body(x + width / 2, y + height / 2);
		Body d = new Body(x + width / 2, y - height / 2);
		
		if (wrappedEdge) 
			physics.addConstraint(new WrappedBoundary(rb, 0, 0, app.getWidth(), app.getHeight()));
		else {
			physics.addConstraint(new Boundary(a, 0, 0, app.getWidth(), app.getHeight()));
			physics.addConstraint(new Boundary(b, 0, 0, app.getWidth(), app.getHeight()));
			physics.addConstraint(new Boundary(c, 0, 0, app.getWidth(), app.getHeight()));
			physics.addConstraint(new Boundary(d, 0, 0, app.getWidth(), app.getHeight()));
		}

		
		Edge e1 = new Edge(a, b, height);
		Edge e2 = new Edge(b, c, width);
		Edge e3 = new Edge(c, d, height);
		Edge e4 = new Edge(d, a, width);

		physics.addConstraint(e1);
		physics.addConstraint(e2);
		physics.addConstraint(e3);
		physics.addConstraint(e4);

		rb.addEdge(e1);
		rb.addEdge(e2);
		rb.addEdge(e3);
		rb.addEdge(e4);

		Link lA = new Link(a, c, (float) Math.sqrt(width * width + height * height));
		Link lB = new Link(b, d, (float) Math.sqrt(width * width + height * height));
		lA.setVisible(false);
		lB.setVisible(false);
		physics.addConstraint(lA);
		physics.addConstraint(lB);

		physics.registerBody(a);
		physics.registerBody(b);
		physics.registerBody(c);
		physics.registerBody(d);

		physics.addConstraint(rb);

		return rb;
	}
	public static void createTriangle (GameContainer app, PhysicsMediator physics, 
			float x, float y, float sideLen) {
		createTriangle(app, physics, x, y, sideLen, false);
	}
	public static void createTriangle (GameContainer app, PhysicsMediator physics, 
			float x, float y, float sideLen, boolean wrappedEdge) {
		Polygon rb = new Polygon();
		float r = sideLen / (float)Math.sqrt(3);
		for (int j = 0; j < 3; j++) {
			Body b = new Body(x + r * (float) Math.cos(j * (2f * Math.PI / 3f)), 
					          y + r * (float) Math.sin(j * (2f * Math.PI / 3f)));
			
			//b.setMass(2000);
			
			physics.registerBody(b);
			
			//physics.addConstraint(new Boundary(b, 0, 0, app.getWidth(), app.getHeight()));
			if (j != 0) {
				Edge e = new Edge(b, physics.getBody(physics.bodyCount() - 2), sideLen);
				rb.addEdge(e);
				physics.addConstraint(e);
			}
			if (j == 2) {
				Edge e = new Edge(b, physics.getBody(physics.bodyCount() - 3), sideLen);
				rb.addEdge(e);
				physics.addConstraint(e);
			}
			if (!wrappedEdge)
				physics.addConstraint(new Boundary(b, 0,0, app.getWidth(), app.getHeight()));
		}
		physics.addConstraint(rb);
		if (wrappedEdge)
			physics.addConstraint(new WrappedBoundary(rb, 0,0, app.getWidth(), app.getHeight()));
	}
	public static void createCircle (GameContainer app, PhysicsMediator physics, float x, float y, float radius) {
		createCircle(app, physics, x, y, radius, false);
	}
	public static void createCircle (GameContainer app, PhysicsMediator physics, float x, float y, float radius, boolean wrappedEdge) {
		Body body = new Body(x,y);
		Circle c = new Circle(body, radius);
		physics.addConstraint(c);
		if (wrappedEdge)
			physics.addConstraint(new WrappedBoundary(c, 0,0, app.getWidth(), app.getHeight()));
		else
			physics.addConstraint(new Boundary(body, radius, radius, app.getWidth() - radius, app.getHeight() - radius));
		physics.registerBody(body);
	}
}
