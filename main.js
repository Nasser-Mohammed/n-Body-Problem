// Three-body simulation with simple Euler integration and collisions

let ctx;
const G = 1; // Gravitational constant scaled for visualization
const dt = 0.1;
let frameCount = 0;
let simulationTime = 0;
let animationId = null;
let running = false;
let currentPlanet = "earth";
let bodies = [];
let particles = [];
let width;
let height;
let isDragging = false;
let planetsInSimulation = [];


/*archived data
const planets = {
  mars: new Image(),
  earth: new Image(),
  jupiter: new Image(),
  saturn: new Image(),
  neptune: new Image(),
  moon: new Image(),
  sun: new Image()
};

planets.moon.src = "images/moon.png";
planets.sun.src = "images/sun.png";
planets.earth.src = "images/earth.png";
planets.jupiter.src = "images/jupiter.png";
planets.saturn.src = "images/saturn.png";
planets.mars.src = "images/mars.png";
planets.neptune.src = "images/neptune.png";

const planetSizes = {
  mars: 40,
  earth: 60,
  jupiter: 120,
  saturn: 95,
  neptune: 80,
  moon: 30,
  sun: 275
};

const planetMasses = {
  mars: 0.107,
  earth: 1,
  jupiter: 317.8,
  saturn: 95.2,
  neptune: 17.1,
  moon: 0.0123,
  sun: 333000
};
*/


const celestialObjects = new Map();

celestialObjects.set('sun', {stateVector: {}, size: 275, mass: 333000, inSimulation: false, image: Object.assign(new Image(), {src: "images/sun.png"})});
celestialObjects.set('earth', {stateVector: {}, size: 60, mass: 1, inSimulation: false, image: Object.assign(new Image(), {src: "images/earth.png"})});
celestialObjects.set('moon', {stateVector: {}, size: 30, mass: 0.0123, inSimulation: false, image: Object.assign(new Image(), {src: "images/moon.png"})});
celestialObjects.set('mars', {sateVector: {}, size: 40, mass: 0.107, inSimulation: false, image: Object.assign(new Image(), {src: "images/mars.png"})});
celestialObjects.set('jupiter', {stateVector: {}, size: 120, mass: 317.8, inSimulation: false, image: Object.assign(new Image(), {src: "images/jupiter.png"})});
celestialObjects.set('saturn', {sateVector: {}, size: 95, mass: 95.2, inSimulation: false, image: Object.assign(new Image(), {src: "images/saturn.png"})});
celestialObjects.set('neptune', {stateVector: {},  size: 80, mass: 17.1, inSimulation: false, image: Object.assign(new Image(), {src: "images/neptune.png"})});


function createCelestialInstance(name) {
  const prototype = celestialObjects.get(name);
  if (!prototype) return null;
  return {
    ...prototype // shallow copy all properties
    // stateVector: { ...stateVector }, // clone or initialize new state vector
    // image: Object.assign(new Image(), { src: prototype.image.src }) // ensure a separate Image instance
  };
}

function drawPlanets(){

  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, width, height);
  for(let i = 0; i < planetsInSimulation.length; i++){
    const planet = planetsInSimulation[i];
    console.log(planet.image.src);
    ctx.drawImage(
      planet.image,
      planet.stateVector.x - Math.floor(planet.size/2),
      planet.stateVector.y - Math.floor(planet.size/2),
      planet.size,
      planet.size
    );

  }
}

function addPlanetToSimulation(celestialBody, x, y) {

  const body = createCelestialInstance(celestialBody);
  const fixedBody = {stateVector: {x, y}, size: body.size, mass: body.mass, inSimulation: true, image: Object.assign(new Image(), {src: body.image.src})};

  console.log(fixedBody);

  // Wait for image to load before drawing
  fixedBody.image.onload = () => {
    planetsInSimulation.push(fixedBody);
    drawPlanets();
  };

  // Optional: fallback in case image fails
  fixedBody.image.onerror = () => {
    console.error("Failed to load image for:", celestialBody);
  };
}


function startSimulation() {
  animate();
}

function resetSimulation() {
  running = false;
  cancelAnimationFrame(animationId);
  simulationTime = 0;
  frameCount = 0;
  particles = [];
  document.getElementById("start-simulation").textContent = "Click to Start Simulation";
}


document.addEventListener("DOMContentLoaded", () => {
  const canvas = document.getElementById("simCanvas");
  ctx = canvas.getContext("2d");
  height = ctx.canvas.height;
  width = ctx.canvas.width;
  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, width, height);

  document.querySelectorAll('#planet-palette img').forEach(img => {
  img.addEventListener('dragstart', (e) => {
    
    e.dataTransfer.setData('text/plain', e.target.dataset.planet);

    e.dataTransfer.setDragImage(e.target, e.target.width/2, e.target.height/2);

  });
});
  canvas.addEventListener('dragover', (e) => e.preventDefault());

  canvas.addEventListener('drop', (e) => {
    e.preventDefault();
    const name = e.dataTransfer.getData('text/plain');

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    console.log('Dropped:', name, " at: ", x, ",", y);
    addPlanetToSimulation(name, x, y);
});
  document.getElementById("simCanvas").addEventListener("mousedown", () =>{
    if (isDragging){
    }
    else{
      isDragging = true;
    }
  });

  document.getElementById("simCanvas").addEventListener("mousemove", () => {
      if (isDragging){
        console.log("dragging on canvas");
      }
  });


   document.getElementById("simCanvas").addEventListener("mouseup", () =>{
    isDragging = false;
  });


  document.getElementById("start-simulation").addEventListener("click", () => {
    const btn = document.getElementById("start-simulation");
    if (!running) {
      running = true;
      btn.textContent = "Pause";
      startSimulation();
    } else {
      running = false;
      cancelAnimationFrame(animationId);
      btn.textContent = "Resume";
    }
  });

  document.getElementById("reset").addEventListener("click", () => {
    const btn = document.getElementById("start-simulation");
    btn.textContent = "Click to Start Simulation";
    resetSimulation();
  });
});