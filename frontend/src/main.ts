import * as THREE from 'three';

interface RobotState {
  actuator1: number;
  actuator2: number;
}

const ws = new WebSocket('ws://localhost:3000');

ws.onopen = () => console.log('Connected to WebSocket server');

ws.onmessage = (event: MessageEvent) => {
  try {
    const data: RobotState = JSON.parse(event.data);
    console.log('Received:', data);
    const actuator1 = document.getElementById('actuator1');
    const actuator2 = document.getElementById('actuator2');
    if (actuator1 && actuator2) {
      actuator1.textContent = data.actuator1.toString();
      actuator2.textContent = data.actuator2.toString();
    }
    // Update arm rotations (Z-axis for XY plane)
    if (arm1 && arm2) {
      arm1.rotation.z = THREE.MathUtils.degToRad(data.actuator1);
      arm2.rotation.z = THREE.MathUtils.degToRad(data.actuator2);
    }
  } catch (error) {
    console.error('Invalid JSON:', error);
  }
};

ws.onerror = (error: Event) => console.error('WebSocket error:', error);
ws.onclose = () => console.log('WebSocket closed');

// Send button handlers
const sendActuator1 = document.getElementById('send-actuator1') as HTMLButtonElement;
const sendActuator2 = document.getElementById('send-actuator2') as HTMLButtonElement;
const inputActuator1 = document.getElementById('input-actuator1') as HTMLInputElement;
const inputActuator2 = document.getElementById('input-actuator2') as HTMLInputElement;

sendActuator1?.addEventListener('click', () => {
  const value = parseFloat(inputActuator1.value);
  if (!isNaN(value)) {
    ws.send(JSON.stringify({ actuator1: value }));
    console.log('Sent:', { actuator1: value });
    // inputActuator1.value = ''; // Clear input
  }
});

sendActuator2?.addEventListener('click', () => {
  const value = parseFloat(inputActuator2.value);
  if (!isNaN(value)) {
    ws.send(JSON.stringify({ actuator2: value }));
    console.log('Sent:', { actuator2: value });
    // inputActuator2.value = ''; // Clear input
  }
});

// Three.js setup

// Create a scene, camera, and renderer
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, 600 / 400, 0.1, 1000);
camera.position.set(3, 1, 2); // 3m right, 3m back, 1m up
camera.lookAt(0, 0, 2); // Look 2m above world origin
camera.rotateZ(THREE.MathUtils.degToRad(90)); // Rotate 90° counterclockwise around Y

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('three-canvas') as HTMLCanvasElement });
renderer.setSize(600, 400);

// Base
const baseGeometry = new THREE.BoxGeometry(0.3, 0.3, 1.5);
baseGeometry.translate(0, 0, 0.75); // Shift origin to bottom of box
const baseMaterial = new THREE.MeshBasicMaterial({ color: 0x888888 });
const base = new THREE.Mesh(baseGeometry, baseMaterial);
scene.add(base);

// Arm 1
const arm1Geometry = new THREE.BoxGeometry(1, 0.2, 0.2);
arm1Geometry.translate(0.5, 0, 0); // Shift origin to end (X=0)
const arm1Material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const arm1 = new THREE.Mesh(arm1Geometry, arm1Material);
arm1.position.set(0, 0, 1.5); // Pivot at base, extend right
scene.add(arm1);

// Arm 2 (attached to Arm 1)
const arm2Geometry = new THREE.BoxGeometry(0.7, 0.2, 0.2);
arm2Geometry.translate(0.35, 0, 0); // Shift origin to end (X=0)
const arm2Material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const arm2 = new THREE.Mesh(arm2Geometry, arm2Material);
arm2.position.set(1, 0, -0.20); // Relative to arm1's reference frame
arm1.add(arm2); // Parent arm2 to arm1

// Animation loop
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();