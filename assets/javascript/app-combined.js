paper.install(window);


//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net

//
//  Ideas for features and optimization

//  IDEA 1
//  Preprocess all the boids and store the distances in a matrix
//  -> this reduces the number of calculations
//  -> give each boid an id to easily find itself in the matrix

//  IDEA 2
//  do all calculations in one loop, reduce duplication

//  IDEA 3
//  use a quadtree or similar to find the nearest neighbour

//  IDEA 4 - DONE
//  let the boids react to the mouse cursor

//  IDEA 5
//  give the boids a limited field of view

//  IDEA 6
//  show the different vectors on one of the boids

//  IDEA 7
//  add wander behavior if boid is not close to anybody else

//  INSPIRATION FOR OPTIMIZATION
//  https://github.com/hughsk/boids
//  https://github.com/jrhdoty/SwarmJS

var Boid = Base.extend({
  initialize: function(x, y) {
    this.position = new Point(x, y);
    this.acceleration = new Point(0, 0);
    this.velocity = new Point(Math.random() * 2 - 1, Math.random() * 2 - 1);

    this.r = 3.0; // wraparound distance - original
    this.maxspeed = 3; // Maximum speed - original 3
    this.maxforce = 0.1; // Maximum steering force - original 0.05
    this.desiredseparation = 25.0; // Desired separation between boids - original 25.0
    this.alignmentneighbordist = 50; // Distance to follow average velocity - original 50
    this.cohesionneighbordist = 200; // Distance to steer to 'center of gravity' - original 50
    this.separationweight = 1.5; // weight of the separation vector - original 1.5
    this.alignmentweight = 1.0; // weight of the alignment vector - original 1.0
    this.cohesionweight = 1.2; // weight of the cohesion vector - original 1.0
    this.avoidweight = 0.1; // weight of the avoid vector - original 1.0
    this.avoidDistance = 100; // distance to stay away from the mouse - original 100

    //
    //  draw base boid arrow
    //
    var arrowLength = 8;
    var arrowSideLength = 8;

    var arrowSide = new Point(arrowSideLength, 0);
    var arrowStart = new Point(0, 0);
    var arrowEnd = arrowStart.subtract(new Point(arrowLength, 0));

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

  run: function(boids, currentMousePos) {
    this.flock(boids, currentMousePos);
    this.update();
    this.borders();
    this.render();
  },

  applyForce: function(force) {
    // We could add mass here if we want A = F / M
    this.acceleration = this.acceleration.add(force);
  },

  flock: function(boids, currentMousePos) {

    var sep = this.separate(boids); // Separation
    var ali = this.alignment(boids); // Alignment
    var coh = this.cohesion(boids); // Cohesion
    var avo = this.avoid(currentMousePos); // Avoid

    //  Arbitrarily weight these forces
    //  OPTIMIZE: do this multiplication together with other multiplications on this vector
    sep = sep.multiply(this.separationweight);
    ali = ali.multiply(this.alignmentweight);
    coh = coh.multiply(this.cohesionweight);
    avo = avo.multiply(this.avoidweight);

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
    var desired = target.subtract(this.position);

    // Normalize desired and scale to maximum speed
    desired = desired.normalize(this.maxspeed);

    var steer = desired.subtract(this.velocity);

    if (steer.length > this.maxforce) {
      steer = steer.normalize(this.maxforce);
    }

    return steer;
  },

  borders: function() {
    var width = view.viewSize.width;
    var height = view.viewSize.height;
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
  separate: function(boids) {

    var steer = new Point(0, 0);
    var count = 0;
    // For every boid in the system, check if it's too close
    var n = boids.length;
    for (var i = 0; i < n; i++) {

      var d = this.position.getDistance(boids[i].position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < this.desiredseparation)) {
        // Calculate vector pointing away from neighbor
        var diff = this.position.subtract(boids[i].position);
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
  alignment: function(boids) {

    var sum = new Point(0, 0);
    var count = 0;
    var n = boids.length;
    for (var i = 0; i < n; i++) {
      // OPTIMIZE: no need to calculate this twice!
      var d = this.position.getDistance(boids[i].position);
      if ((d > 0) && (d < this.alignmentneighbordist)) {
        sum = sum.add(boids[i].velocity);
        count++;
      }
    }

    if (count > 0) {
      // OPTIMIZE: this code is repeated in other places
      sum = sum.divide(count);
      sum = sum.normalize(this.maxspeed);
      var steer = sum.subtract(this.velocity);
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
  cohesion: function(boids) {

    var sum = new Point(0, 0); // Start with empty vector to accumulate all locations
    var count = 0;
    var n = boids.length;

    for (var i = 0; i < n; i++) {
      var d = this.position.getDistance(boids[i].position);
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
    var ap = currentMousePos.subtract(this.position);
    var apLength = ap.length;


    // OPTIMIZE: make strength of avoidVector correlate to distance from mouse
    if (apLength < this.avoidDistance) {
      var ab = this.position.add(this.velocity);

      ab = ab.normalize();
      ab = ab.multiply(ap.dot(ab));

      var avoidVector = ab.subtract(ap).multiply(1-(apLength/this.avoidDistance));

      return avoidVector.normalize();
    } else {
      return new Point(0, 0);
    }
  }
});


//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net


var Flock = Base.extend({
  initialize: function() {
    this.boids = []; // Initialize array to hold the boids
    this.l = 0; // Track the number of boids
    this.currentMousePos = new Point();
  },

  addBoid: function(newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function() {
    for (var i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids, this.currentMousePos);  // Passing the entire list of boids to each boid individually
    }
  },

  updateMouse: function(mousePos) {
    this.currentMousePos = mousePos;
  }
});


//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net

function startPaper() {
  paper.setup('canvas');

  var nrBoids = 60;

  // Create a new flock
  flock = new Flock();

  // Add an initial set of boids into the system
  for (var i = 0; i < nrBoids; i++) {
    var x = Math.random()*view.viewSize.width;
    var y = Math.random()*view.viewSize.height;
    var b = new Boid(x,y);
    flock.addBoid(b);
  }

  //
  //  onFrame
  //
  view.onFrame = function(event) {
    flock.run();
  }

  view.onMouseMove = function(event) {
    flock.updateMouse(event.point);
  }

}

//
//  make sure things get started at the right time
//
function addOnloadListener(func) {
  if (window.addEventListener)
    window.addEventListener('load', func, false);
  else if (window.attachEvent)
    window.attachEvent('onload', func);
  else window.onload = chain(window.onload, func);
}

addOnloadListener(startPaper);


//
//  This statements below are used by Codekit to combine multiple javascript files into a single file.
//  See https://codekitapp.com for more information.
//
//  If you don't use Codekit you can ignore this and use the separate javascript files.
//  Include them in the html source in the same order as below.

// @codekit-prepend "core.js";
// @codekit-prepend "boid.js";
// @codekit-prepend "flock.js";
// @codekit-prepend "sketch.js";


