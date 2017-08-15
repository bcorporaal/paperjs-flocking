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
    this.frameCounter = 0;

    //  number of frames that the distance calculation is skipped
    this.frameSkip = 3;
    //  increase with 1 to make the modulo calculation easier
    this.frameSkip++;
  },

  addBoid: function(newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function() {

    //  calculate an array with all the squared distances (so not each boid has to do that individually)
    //  skip a defined number of frames
    if ((this.frameCounter%this.frameSkip) == 0) {
      let d = 0;
      for (let i = 0; i < this.l; i++) {
        this.distances[i] = [];
        this.distances[i][i] = 0;
        for (let j = 0; j < i; j++) {
          d = this.boids[i].position.getDistance(this.boids[j].position, true);
          this.distances[i][j] = d;
          this.distances[j][i] = d;
        }
      }
    }

    //  move each individual boid
    for (let i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids, this.currentMousePos, this.distances);  // Passing the entire list of boids to each boid individually
    }

    this.frameCounter++;
  },

  updateMouse: function(mousePos) {
    this.currentMousePos = mousePos;
  }
});
