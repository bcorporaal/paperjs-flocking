//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net


var Flock = Base.extend({
  initialize: function() {
    this.boids = []; // Initialize array to hold the boids
    this.l = 0; // Track the number of boids
  },

  addBoid: function(newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function() {
    for (var i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
    }
  }
});
