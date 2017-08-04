//  Flocking system
//  Based on chapter 6 of Nature of Code by Daniel Shiffman
//  http://natureofcode.com/book/chapter-6-autonomous-agents/
//
//  Rewritten for PaperJS and optimized by Bob Corporaal - https://reefscape.net

function startPaper() {
  paper.setup('canvas');
  let nrBoids = 60;

  // Create a new flock
  flock = new Flock();

  // Add an initial set of boids into the system
  for (let i = 0; i < nrBoids; i++) {

    //
    //  get a random point on the perimiter
    //
    let frame = 5; // off screen margin for the boids
    let w = view.viewSize.width+2*frame;
    let h = view.viewSize.height+2*frame;
    let x = 0;
    let y = 0;

    //
    //  get random point on the perimiter
    //  taken from https://stackoverflow.com/questions/9005750/generate-a-random-point-on-a-rectangles-perimeter-with-uniform-distribution
    //
    let r = Math.random();
    x = w*Math.min(1, Math.max(0, Math.abs((r * 4 - .5) % 4 - 2) - .5))-frame;
    y = h*Math.min(1, Math.max(0, Math.abs((r * 4 + .5) % 4 - 2) - .5))-frame;
    let b = new Boid(x,y);
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
