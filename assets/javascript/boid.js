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

//  IDEA 2
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
    const fnoise = 0.35;

    // wraparound distance - original 3.0
    this.r = 10;

    // Maximum speed - original 3
    this.maxspeed = this.addNoise(1.6, fnoise);

    //  Give a random starting velocity based on maxspeed
    const startVelocity = 0.25;
    this.velocity = new Point(startVelocity * this.maxspeed * (1 - 2 * Math.random()),
                              startVelocity * this.maxspeed * (1 - 2 * Math.random()));

    // Maximum steering force - original 0.05
    this.maxforce = this.addNoise(0.04, fnoise);

    // Desired separation between boids - original 25.0
    this.desiredseparation = 25.0; // not random to ensure boids keep some distance

    // Distance to follow average velocity - original 50
    this.alignmentneighbordist = this.addNoise(50, fnoise);

    // Distance to steer to 'center of gravity' - original 50
    this.cohesionneighbordist = this.addNoise(200, fnoise);

    // weight of the separation vector - original 1.5
    this.separationweight = 2; // not random to ensure boids always keep some distance

    // weight of the alignment vector - original 1.0
    this.alignmentweight = this.addNoise(1.0, fnoise);

    // weight of the cohesion vector - original 1.0
    this.cohesionweight = this.addNoise(1.2, fnoise);

    // weight of the avoid vector - original 1.0
    this.avoidweight = this.addNoise(0.1, fnoise);

    // distance to stay away from the mouse - original 100
    this.avoidDistance = this.addNoise(100, fnoise);

    // mass - original 1
    this.mass = this.addNoise(1, fnoise);

    //
    //  draw base boid arrow
    //
    const arrowLength = 6;
    const arrowSideLength = 6;

    let arrowSide = new Point(arrowSideLength, 0);
    let arrowStart = new Point(0, 0);
    let arrowEnd = arrowStart.subtract(new Point(arrowLength, 0));

    this.arrow = new Group([
      new Path([arrowEnd, arrowStart]),
      new Path([
        arrowStart.add(arrowSide.rotate(135)),
        arrowStart,
        arrowStart.add(arrowSide.rotate(-135))
      ])
    ]);

    this.arrow.strokeWidth = 1;
    this.arrow.strokeColor = '#1C1A20';
    this.arrow.applyMatrix = false;
  },

  addNoise: function(parameter, fnoise) {
    return parameter * (1 + fnoise * (1 - 2 * Math.random()));
  },

  run: function(boids, currentMousePos, distances) {
    this.flock2(boids, currentMousePos, distances);
    this.update();
    this.borders();
    this.render();
  },

  applyForce: function(force) {
    this.acceleration = this.acceleration.add(force);
  },

  flock: function(boids, currentMousePos, distances) {

    let sep = this.separate(boids, distances); // Separation
    let ali = this.alignment(boids, distances); // Alignment
    let coh = this.cohesion(boids, distances); // Cohesion
    let avo = this.avoid(currentMousePos); // Avoid

    //  Arbitrarily weight these forces
    //  OPTIMIZE: do this multiplication together with other multiplications on this vector
    sep = sep.multiply(this.separationweight / this.mass);
    ali = ali.multiply(this.alignmentweight / this.mass);
    coh = coh.multiply(this.cohesionweight / this.mass);
    avo = avo.multiply(this.avoidweight / this.mass);

    // Add the force vectors to acceleration
    // OPTIMIZE: there is no need for a separate function for this
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
    this.applyForce(avo);
  },

  update: function() {

    // Update velocity
    this.velocity = this.velocity.add(this.acceleration);

    // Limit speed
    if (this.velocity.length > this.maxspeed) {
      this.velocity = this.velocity.normalize(this.maxspeed);
    }

    this.position = this.position.add(this.velocity);

    // Reset acceleration to 0 each cycle
    this.acceleration = new Point(0, 0);
  },

  seek: function(target) {
    // A vector pointing from the location to the target
    let desired = target.subtract(this.position);

    // Normalize desired and scale to maximum speed
    desired = desired.normalize(this.maxspeed);

    let steer = desired.subtract(this.velocity);

    if (steer.length > this.maxforce) {
      steer = steer.normalize(this.maxforce);
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

  // Separation
  // Method checks for nearby boids and steers away
  separate: function(boids, distances) {

    let steer = new Point(0, 0);
    let count = 0;
    // For every boid in the system, check if it's too close
    let n = boids.length;
    for (let i = 0; i < n; i++) {

      let d = distances[this.id][i];

      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < this.desiredseparation)) {
        // Calculate vector pointing away from neighbor
        let diff = this.position.subtract(boids[i].position);
        diff = diff.normalize();
        diff = diff.divide(d); // Weight by distance
        steer = steer.add(diff);
        count++; // Keep track of how many are too close
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer = steer.divide(count);
    }

    // As long as the vector is greater than 0
    if (steer.length > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer = steer.normalize();
      steer = steer.multiply(this.maxspeed);
      steer = steer.subtract(this.velocity);
      if (steer.length > this.maxforce) {
        steer = steer.normalize(this.maxforce);
      }
    }
    return steer;
  },

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  alignment: function(boids, distances) {

    let sum = new Point(0, 0);
    let count = 0;
    let n = boids.length;
    for (let i = 0; i < n; i++) {

      let d = distances[this.id][i];

      if ((d > 0) && (d < this.alignmentneighbordist)) {
        sum = sum.add(boids[i].velocity);
        count++;
      }
    }

    if (count > 0) {
      // OPTIMIZE: this code is repeated in other places
      sum = sum.divide(count);
      sum = sum.normalize(this.maxspeed);
      let steer = sum.subtract(this.velocity);
      if (steer.length > this.maxforce) {
        steer = steer.normalize(this.maxforce);
      }
      return steer;
    } else {
      return new Point(0, 0); // OPTIMIZE: If count == 0 then steer should also be 0,0 so always send steer
    }
  },

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  cohesion: function(boids, distances) {

    let sum = new Point(0, 0); // Start with empty vector to accumulate all locations
    let count = 0;
    let n = boids.length;

    for (let i = 0; i < n; i++) {

      let d = distances[this.id][i];

      if ((d > 0) && (d < this.cohesionneighbordist)) {
        sum = sum.add(boids[i].position); // Add location
        count++;
      }
    }

    if (count > 0) {
      sum = sum.divide(count);
      return this.seek(sum); // Steer towards the location
    } else {
      return new Point();
    }

  },

  //  Avoid
  //  Stay away from the current mouse position
  avoid: function(currentMousePos) {
    let ap = currentMousePos.subtract(this.position);
    let apLength = ap.length;


    // OPTIMIZE: make strength of avoidVector correlate to distance from mouse
    if (apLength < this.avoidDistance) {
      let ab = this.position.add(this.velocity);

      ab = ab.normalize();
      ab = ab.multiply(ap.dot(ab));

      let avoidVector = ab.subtract(ap).multiply(1 - (apLength / this.avoidDistance));

      return avoidVector.normalize();
    } else {
      return new Point(0, 0);
    }
  },

  //
  //  Flock 2 - all influences in a single loop
  //
  flock2: function(boids, currentMousePos, distances) {
    let sep = new Point(0, 0); //this.separate(boids, distances); // Separation
    let ali = new Point(0, 0); //this.alignment(boids, distances); // Alignment
    let coh = new Point(0, 0); //this.cohesion(boids, distances); // Cohesion
    let avo = new Point(0, 0); //this.avoid(currentMousePos); // Avoid

    let separationSteer = new Point(0, 0);
    let alignmentSum = new Point(0, 0);
    let separationCount = 0;
    let alignmentCount = 0;
    let cohesionSum = new Point(0, 0);
    let cohesionCount = 0;
    let n = boids.length;
    //
    //  loop all boids
    //
    for (let i = 0; i < n; i++) {

      let d = distances[this.id][i];

      //
      //    separation
      //
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < this.desiredseparation)) {
        // Calculate vector pointing away from neighbor
        let diff = this.position.subtract(boids[i].position);
        diff = diff.normalize();
        diff = diff.divide(d); // Weight by distance
        separationSteer = separationSteer.add(diff);
        separationCount++; // Keep track of how many are too close
      }

      //
      //    alignment
      //
      if ((d > 0) && (d < this.alignmentneighbordist)) {
        alignmentSum = alignmentSum.add(boids[i].velocity);
        alignmentCount++;
      }

      //
      //    cohesion
      //
      if ((d > 0) && (d < this.cohesionneighbordist)) {
        cohesionSum = cohesionSum.add(boids[i].position); // Add location
        cohesionCount++;
      }
    }

    //
    //  process the results from the loop
    //

    //
    //  separation
    //
    if (separationCount > 0) {
      separationSteer = separationSteer.divide(separationCount);

      // Implement Reynolds: Steering = Desired - Velocity
      separationSteer = separationSteer.normalize();
      separationSteer = separationSteer.multiply(this.maxspeed);
      separationSteer = separationSteer.subtract(this.velocity);
      if (separationSteer.length > this.maxforce) {
        separationSteer = separationSteer.normalize(this.maxforce);
      }
    }
    sep = separationSteer;

    //
    //  alignment
    //
    if (alignmentCount > 0) {
      // OPTIMIZE: this code is repeated in other places
      alignmentSum = alignmentSum.divide(alignmentCount);
      alignmentSum = alignmentSum.normalize(this.maxspeed);
      let alignmentSteer = alignmentSum.subtract(this.velocity);
      if (alignmentSteer.length > this.maxforce) {
        alignmentSteer = alignmentSteer.normalize(this.maxforce);
      }
      ali = alignmentSteer;
    } else {
      ali = new Point(0, 0); // OPTIMIZE: If count == 0 then steer should also be 0,0 so always send steer
    }

    //
    //  cohesion
    //
    if (cohesionCount > 0) {
      cohesionSum = cohesionSum.divide(cohesionCount);
      coh = this.seek(cohesionSum); // Steer towards the location
    } else {
      coh = new Point();
    }

    //
    //  avoid
    //
    let ap = currentMousePos.subtract(this.position);
    let apLength = ap.length;

    // OPTIMIZE: make strength of avoidVector correlate to distance from mouse
    if (apLength < this.avoidDistance) {
      let ab = this.position.add(this.velocity);

      ab = ab.normalize();
      ab = ab.multiply(ap.dot(ab));

      let avoidVector = ab.subtract(ap).multiply(1 - (apLength / this.avoidDistance));

      avo = avoidVector.normalize();
    } else {
      avo = new Point(0, 0);
    }

    //  Arbitrarily weight these forces
    //  OPTIMIZE: do this multiplication together with other multiplications on this vector
    sep = sep.multiply(this.separationweight / this.mass);
    ali = ali.multiply(this.alignmentweight / this.mass);
    coh = coh.multiply(this.cohesionweight / this.mass);
    avo = avo.multiply(this.avoidweight / this.mass);

    // Add the force vectors to acceleration
    // OPTIMIZE: there is no need for a separate function for this
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
    this.applyForce(avo);
  }
});
