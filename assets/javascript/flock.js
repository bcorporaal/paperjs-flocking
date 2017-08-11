//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net




let Flock = Base.extend({
  initialize: function() {
    this.boids = []; // Initialize array to hold the boids
    this.l = 0; // Track the number of boids
    this.currentMousePos = new Point();
    this.distances = []; // array with the distance between all the boids
  },

  addBoid: function(newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function() {
    //  calculate an array with all the distances (so not each boid has to do that individually)
    //  OPTIMIZE: use squared distance to remove one calculation
    let d = 0;
    for (let i = 0; i < this.l; i++) {
      this.distances[i] = [];
      this.distances[i][i] = 0;
      for (let j = 0; j < i; j++) {
        d = this.boids[i].position.getDistance(this.boids[j].position);
        this.distances[i][j] = d;
        this.distances[j][i] = d;
      }
    }

    //  move each individual boid
    for (let i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids, this.currentMousePos, this.distances);  // Passing the entire list of boids to each boid individually
    }
  },

  updateMouse: function(mousePos) {
    this.currentMousePos = mousePos;
  }
});
