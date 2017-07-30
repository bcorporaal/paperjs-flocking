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

  // view.onMouseMove = function(event) {
  //   mousePos = event.point;
  // }

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
console.log('wait…');
