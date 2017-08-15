//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net

//
//  Ideas for features and optimization

//  IDEA 1 - DONE
//  Preprocess all the boids and store the distances in a matrix
//  -> this reduces the number of calculations
//  -> give each boid an id to easily find itself in the matrix

//  IDEA 2 - DONE
//  do all calculations in one loop, reduce duplication

//  IDEA 3
//  use a quadtree or similar to find the nearest neighbour

//  IDEA 4 - DONE
//  let the boids react to the mouse cursor

//  IDEA 5 - DONE and removed again - it is not beneficial
//  give the boids a limited field of view

//  IDEA 6
//  show the different vectors on one of the boids

//  IDEA 7
//  add wander behavior if boid is not close to anybody else

//  IDEA 8
//  confine to screen instead of wrapping around

//  INSPIRATION FOR OPTIMIZATION
//  https://github.com/hughsk/boids
//  https://github.com/jrhdoty/SwarmJS

let Boid = Base.extend({
  initialize: function(id, x, y) {
    this.position = new Point(x, y);
    this.acceleration = new Point(0, 0);
    this.id = id;

    // randomnessfnoise
    const fnoise = 0.40;

    // wraparound distance - original 3.0
    this.r = 10;

    // Maximum speed - original 3
    this.maxSpeed = this.addNoise(1.7, fnoise);

    //  Give a random starting velocity based on maxSpeed
    const startVelocity = 0.25;
    this.velocity = new Point(startVelocity * this.maxSpeed * (1 - 2 * Math.random()),
                              startVelocity * this.maxSpeed * (1 - 2 * Math.random()));

    // Maximum steering force - original 0.05
    this.maxForce = this.addNoise(0.06, fnoise);

    // Desired separation between boids - original 25.0
    this.desiredSeparation = 30.0; // not random to ensure boids keep some distance

    // Distance to follow average velocity - original 50
    this.alignmentNeighborDist = this.addNoise(50, fnoise);

    // Distance to steer to 'center of gravity' - original 50
    this.cohesionNeighborDist = this.addNoise(200, fnoise);

    // weight of the separation vector - original 1.5
    this.separationWeight = 2; // not random to ensure boids always keep some distance

    // weight of the alignment vector - original 1.0
    this.alignmentWeight = this.addNoise(1.0, fnoise);

    // weight of the cohesion vector - original 1.0
    this.cohesionWeight = this.addNoise(1.2, fnoise);

    // weight of the avoid vector - original 1.0
    this.avoidWeight = this.addNoise(0.1, fnoise);

    // distance to stay away from the mouse - original 100
    this.avoidDistance = this.addNoise(100, fnoise);

    // mass - original 1
    this.mass = this.addNoise(1, fnoise);

    //
    //  draw base boid arrow
    //
    const arrowLength = 7;
    const arrowSideLength = 7;

    let arrowSide = new Point(arrowSideLength, 0);
    let arrowStart = new Point(0, 0);
    let arrowEnd = arrowStart.subtract(new Point(arrowLength, 0));

    this.arrowOriginal = new Group([
      new Path([arrowEnd, arrowStart]),
      new Path([
        arrowStart.add(arrowSide.rotate(135)),
        arrowStart,
        arrowStart.add(arrowSide.rotate(-135))
      ])
    ]);

    this.arrowOriginal.strokeWidth = 1;
    this.arrowOriginal.strokeColor = '#1C1A20';
    this.arrowOriginal.applyMatrix = false;

    this.arrow = this.arrowOriginal.rasterize();
    this.arrowOriginal.remove();
  },

  addNoise: function(parameter, fnoise) {
    return parameter * (1 + fnoise * (1 - 2 * Math.random()));
  },

  run: function(boids, currentMousePos, distances) {
    this.flockStep(boids, currentMousePos, distances);
    this.update();
    this.borders();
    this.render();
  },

  update: function() {

    // Update velocity
    this.velocity = this.velocity.add(this.acceleration);

    // Limit speed
    if (this.velocity.length > this.maxSpeed) {
      this.velocity = this.velocity.normalize(this.maxSpeed);
    }

    this.position = this.position.add(this.velocity);

    // Reset acceleration to 0 each cycle
    this.acceleration = new Point(0, 0);
  },

  seek: function(target) {
    // A vector pointing from the location to the target
    let desired = target.subtract(this.position);

    // Normalize desired and scale to maximum speed
    desired = desired.normalize(this.maxSpeed);

    let steer = desired.subtract(this.velocity);

    if (steer.length > this.maxForce) {
      steer = steer.normalize(this.maxForce);
    }

    return steer;
  },

  borders: function() {
    let width = view.viewSize.width;
    let height = view.viewSize.height;
    if (this.position.x < -this.r) this.position.x = width + this.r;
    if (this.position.y < -this.r) this.position.y = height + this.r;
    if (this.position.x > (width + this.r)) this.position.x = -this.r;
    if (this.position.y > (height + this.r)) this.position.y = -this.r;
  },

  render: function() {
    this.arrow.position = this.position;
    this.arrow.rotate(this.velocity.angle - this.arrow.rotation);
  },

  //
  //  flockStep - all boid influences in a single loop
  //
  flockStep: function(boids, currentMousePos, distances) {
    let sepVector = new Point(0, 0); // Separation
    let aliVector = new Point(0, 0); // Alignment
    let cohVector = new Point(0, 0); // Cohesion
    let avoVector = new Point(0, 0); // Avoid

    let sepCount = 0;
    let aliSum = new Point(0, 0);
    let aliCount = 0;
    let cohSum = new Point(0, 0);
    let cohCount = 0;

    let n = boids.length;

    //
    //  loop all boids
    //
    for (let i = 0; i < n; i++) {

      let d = distances[this.id][i];

      //  separation - loop
      if ((d > 0) && (d < this.desiredSeparation)) {
        // calculate vector pointing away from neighbor
        let diff = this.position.subtract(boids[i].position);
        diff = diff.normalize();
        diff = diff.divide(d); // weight by distance
        sepVector = sepVector.add(diff);
        sepCount++; // count how many are too close
      }

      //  alignment - loop
      if ((d > 0) && (d < this.alignmentNeighborDist)) {
        aliSum = aliSum.add(boids[i].velocity);
        aliCount++;
      }

      //  cohesion - loop
      if ((d > 0) && (d < this.cohesionNeighborDist)) {
        cohSum = cohSum.add(boids[i].position); // add location
        cohCount++; // count how many the boid tries to stay close to
      }
    }

    //  process the results from the loop

    //  separation - results
    if (sepCount > 0) {
      sepVector = sepVector.divide(sepCount);

      // Implement Reynolds: Steering = Desired - Velocity
      sepVector = sepVector.normalize();
      sepVector = sepVector.multiply(this.maxSpeed);
      sepVector = sepVector.subtract(this.velocity);
      if (sepVector.length > this.maxForce) {
        sepVector = sepVector.normalize(this.maxForce);
      }
    }

    //  alignment - results
    if (aliCount > 0) {
      aliSum = aliSum.divide(aliCount);
      aliSum = aliSum.normalize(this.maxSpeed);

      aliVector = aliSum.subtract(this.velocity);
      if (aliVector.length > this.maxForce) {
        aliVector = aliVector.normalize(this.maxForce);
      }
    }

    //  cohesion - results
    if (cohCount > 0) {
      cohSum = cohSum.divide(cohCount);
      cohVector = this.seek(cohSum); // Steer towards the location
    }

    //  avoid - results
    let ap = currentMousePos.subtract(this.position);
    let apLength = ap.length;

    // OPTIMIZE: make strength of avoidVector correlate to distance from mouse
    if (apLength < this.avoidDistance) {
      let ab = this.position.add(this.velocity);

      ab = ab.normalize();
      ab = ab.multiply(ap.dot(ab));

      avoVector = ab.subtract(ap).multiply(1 - (apLength / this.avoidDistance));
      avoVector = avoVector.normalize();
    }

    //
    //  combine
    //

    //  weigh these forces and apply the mass of the boid
    sepVector = sepVector.multiply(this.separationWeight / this.mass);
    aliVector = aliVector.multiply(this.alignmentWeight / this.mass);
    cohVector = cohVector.multiply(this.cohesionWeight / this.mass);
    avoVector = avoVector.multiply(this.avoidWeight / this.mass);

    // add the force vectors to acceleration
    this.acceleration = this.acceleration.add(sepVector);
    this.acceleration = this.acceleration.add(aliVector);
    this.acceleration = this.acceleration.add(cohVector);
    this.acceleration = this.acceleration.add(avoVector);
  }
});
