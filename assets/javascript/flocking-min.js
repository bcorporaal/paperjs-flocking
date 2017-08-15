paper.install(window);


let Boid = Base.extend({
  initialize: function (id, x, y) {
    this.position = new Point(x, y);
    this.acceleration = new Point(0, 0);
    this.id = id;

    const fnoise = 0.35;

    this.r = 10;

    this.maxSpeed = this.addNoise(1.6, fnoise);

    const startVelocity = 0.25;
    this.velocity = new Point(startVelocity * this.maxSpeed * (1 - 2 * Math.random()), startVelocity * this.maxSpeed * (1 - 2 * Math.random()));

    this.maxForce = this.addNoise(0.04, fnoise);

    this.desiredSeparation = 25.0;
    this.alignmentNeighborDist = this.addNoise(50, fnoise);

    this.cohesionNeighborDist = this.addNoise(200, fnoise);

    this.separationWeight = 2;
    this.alignmentWeight = this.addNoise(1.0, fnoise);

    this.cohesionWeight = this.addNoise(1.2, fnoise);

    this.avoidWeight = this.addNoise(0.1, fnoise);

    this.avoidDistance = this.addNoise(100, fnoise);

    this.mass = this.addNoise(1, fnoise);

    const arrowLength = 6,
          arrowSideLength = 6;


    let arrowSide = new Point(arrowSideLength, 0),
        arrowStart = new Point(0, 0),
        arrowEnd = arrowStart.subtract(new Point(arrowLength, 0));


    this.arrow = new Group([new Path([arrowEnd, arrowStart]), new Path([arrowStart.add(arrowSide.rotate(135)), arrowStart, arrowStart.add(arrowSide.rotate(-135))])]);

    this.arrow.strokeWidth = 1;
    this.arrow.strokeColor = '#1C1A20';
    this.arrow.applyMatrix = !1;
  },

  addNoise: function (parameter, fnoise) {
    return parameter * (1 + fnoise * (1 - 2 * Math.random()));
  },

  run: function (boids, currentMousePos, distances) {
    this.flockStep(boids, currentMousePos, distances);
    this.update();
    this.borders();
    this.render();
  },

  update: function () {
    this.velocity = this.velocity.add(this.acceleration);

    if (this.velocity.length > this.maxSpeed) {
      this.velocity = this.velocity.normalize(this.maxSpeed);
    }

    this.position = this.position.add(this.velocity);

    this.acceleration = new Point(0, 0);
  },

  seek: function (target) {
    let desired = target.subtract(this.position);

    desired = desired.normalize(this.maxSpeed);

    let steer = desired.subtract(this.velocity);

    if (steer.length > this.maxForce) {
      steer = steer.normalize(this.maxForce);
    }

    return steer;
  },

  borders: function () {
    let width = view.viewSize.width,
        height = view.viewSize.height;

    if (this.position.x < -this.r) this.position.x = width + this.r;
    if (this.position.y < -this.r) this.position.y = height + this.r;
    if (this.position.x > width + this.r) this.position.x = -this.r;
    if (this.position.y > height + this.r) this.position.y = -this.r;
  },

  render: function () {
    this.arrow.position = this.position;
    this.arrow.rotate(this.velocity.angle - this.arrow.rotation);
  },

  flockStep: function (boids, currentMousePos, distances) {
    let sepVector = new Point(0, 0),
        aliVector = new Point(0, 0),
        cohVector = new Point(0, 0),
        avoVector = new Point(0, 0),
        sepCount = 0,
        aliSum = new Point(0, 0),
        aliCount = 0,
        cohSum = new Point(0, 0),
        cohCount = 0,
        n = boids.length;
    for (let i = 0, d; i < n; i++) {
      d = distances[this.id][i];

      if (d > 0 && d < this.desiredSeparation) {
        let diff = this.position.subtract(boids[i].position);
        diff = diff.normalize();
        diff = diff.divide(d);
        sepVector = sepVector.add(diff);
        sepCount++;
      }

      if (d > 0 && d < this.alignmentNeighborDist) {
        aliSum = aliSum.add(boids[i].velocity);
        aliCount++;
      }

      if (d > 0 && d < this.cohesionNeighborDist) {
        cohSum = cohSum.add(boids[i].position);
        cohCount++;
      }
    }

    if (sepCount > 0) {
      sepVector = sepVector.divide(sepCount);

      sepVector = sepVector.normalize();
      sepVector = sepVector.multiply(this.maxSpeed);
      sepVector = sepVector.subtract(this.velocity);
      if (sepVector.length > this.maxForce) {
        sepVector = sepVector.normalize(this.maxForce);
      }
    }

    if (aliCount > 0) {
      aliSum = aliSum.divide(aliCount);
      aliSum = aliSum.normalize(this.maxSpeed);

      aliVector = aliSum.subtract(this.velocity);
      if (aliVector.length > this.maxForce) {
        aliVector = aliVector.normalize(this.maxForce);
      }
    }

    if (cohCount > 0) {
      cohSum = cohSum.divide(cohCount);
      cohVector = this.seek(cohSum);
    }

    let ap = currentMousePos.subtract(this.position),
        apLength = ap.length;

    if (apLength < this.avoidDistance) {
      let ab = this.position.add(this.velocity);

      ab = ab.normalize();
      ab = ab.multiply(ap.dot(ab));

      avoVector = ab.subtract(ap).multiply(1 - apLength / this.avoidDistance);
      avoVector = avoVector.normalize();
    }

    sepVector = sepVector.multiply(this.separationWeight / this.mass);
    aliVector = aliVector.multiply(this.alignmentWeight / this.mass);
    cohVector = cohVector.multiply(this.cohesionWeight / this.mass);
    avoVector = avoVector.multiply(this.avoidWeight / this.mass);

    this.acceleration = this.acceleration.add(sepVector);
    this.acceleration = this.acceleration.add(aliVector);
    this.acceleration = this.acceleration.add(cohVector);
    this.acceleration = this.acceleration.add(avoVector);
  }
});


let Flock = Base.extend({
  initialize: function () {
    this.boids = [];
    this.l = 0;
    this.currentMousePos = new Point();
    this.distances = [];
  },

  addBoid: function (newBoid) {
    this.l++;
    this.boids.push(newBoid);
  },

  run: function () {
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

    for (let i = 0; i < this.l; i++) {
      this.boids[i].run(this.boids, this.currentMousePos, this.distances);
    }
  },

  updateMouse: function (mousePos) {
    this.currentMousePos = mousePos;
  }
});


function startPaper() {
  paper.setup('boid-canvas');
  const nrBoids = 80;

  flock = new Flock();

  for (let i = 0; i < nrBoids; i++) {
    const frame = 5;
    let w = view.viewSize.width + 2 * frame,
        h = view.viewSize.height + 2 * frame,
        x = 0,
        y = 0,
        r = Math.random();

    x = w * Math.min(1, Math.max(0, Math.abs((r * 4 - .5) % 4 - 2) - .5)) - frame;
    y = h * Math.min(1, Math.max(0, Math.abs((r * 4 + .5) % 4 - 2) - .5)) - frame;
    let b = new Boid(i, x, y);
    flock.addBoid(b);
  }

  view.onFrame = function (event) {
    flock.run();
  };

  view.onMouseMove = function (event) {
    flock.updateMouse(event.point);
  };
}

function addOnloadListener(func) {
  if (window.addEventListener) window.addEventListener('load', func, !1);else if (window.attachEvent) window.attachEvent('onload', func);else window.onload = chain(window.onload, func);
}

addOnloadListener(startPaper);

