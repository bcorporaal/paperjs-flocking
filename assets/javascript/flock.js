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
  },

  addBoid: function(newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function() {
    for (let i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids, this.currentMousePos);  // Passing the entire list of boids to each boid individually
    }
  },

  updateMouse: function(mousePos) {
    this.currentMousePos = mousePos;
  }
});
