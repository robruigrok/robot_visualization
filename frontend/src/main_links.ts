import * as THREE from 'three';

interface LinkState {
  link_name: string;
  movable: string;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number };
}

interface GoalPose {
  x: number;
  y: number;
  z: number;
  rotz: number;
}

const ws = new WebSocket('ws://localhost:3000');
const stateDiv = document.getElementById('state')!;
const controlsDiv = document.getElementById('controls')!;
const scene = new THREE.Scene();
const meshes: THREE.Mesh[] = []; // Store link meshes
let links: LinkState[] = []; // Store links for Send All
let controlsInitialized = false; // Track if controls are created

// WebSocket handlers
ws.onopen = () => console.log('Connected to WebSocket server');

ws.onmessage = (event: MessageEvent) => {
  try {
    links = JSON.parse(event.data); // Store links globally
    console.log('Received:', links);

    // Clear previous UI
    stateDiv.innerHTML = '';

    // Create UI for each link
    links.forEach((link, index) => {
      // State display
      const stateP = document.createElement('p');
      let stateText: string;
      switch (link.movable) {
        case 'STATIC':
          stateText = 'Fixed';
          break;
        case 'X':
          stateText = `${link.translation.x.toFixed(2)} m`;
          break;
        case 'Y':
          stateText = `${link.translation.y.toFixed(2)} m`;
          break;
        case 'Z':
          stateText = `${link.translation.z.toFixed(2)} m`;
          break;
        case 'ROT_X':
          stateText = `${THREE.MathUtils.radToDeg(link.rotation.x).toFixed(1)} deg`;
          break;
        case 'ROT_Y':
          stateText = `${THREE.MathUtils.radToDeg(link.rotation.y).toFixed(1)} deg`;
          break;
        case 'ROT_Z':
          stateText = `${THREE.MathUtils.radToDeg(link.rotation.z).toFixed(1)} deg`;
          break;
        default:
          stateText = 'Unknown';
      }
      if (link.movable !== 'Move_Base') { // Avoid displaying movable base
        stateP.innerHTML = `${link.link_name}: <span id="state-${link.link_name}">${stateText}</span>`;
        stateDiv.appendChild(stateP);
      }
    });


    // Generate controls only once
    if (!controlsInitialized) {
      links.forEach((link, index) => {
        if (link.movable !== 'STATIC' && link.movable !== 'Move_Base') {
          const controlP = document.createElement('p');
          // make distinction between translation and rotation
          if (link.movable === 'X' || link.movable === 'Y' || link.movable === 'Z') {
              controlP.innerHTML = `
              Set ${link.link_name}:
              <input type="number" id="input-${link.link_name}" placeholder="Enter distance (m)" step="0.01" />
              <button id="send-${link.link_name}">Send</button>
              `;
          }
          else {
              controlP.innerHTML = `
              Set ${link.link_name}: 
              <input type="number" id="input-${link.link_name}" placeholder="Enter angle (deg)" step="1.0" />
              <button id="send-${link.link_name}">Send</button>
              `;
          }
          // controlP.innerHTML = `
          //   Set ${link.link_name}: 
          //   <input type="number" id="input-${link.link_name}" placeholder="Enter angle (deg)" step="1.0" />
          //   <button id="send-${link.link_name}">Send</button>
          // `;
          controlsDiv.appendChild(controlP);

          // Add button handler
          const button = document.getElementById(`send-${link.link_name}`) as HTMLButtonElement;
          const input = document.getElementById(`input-${link.link_name}`) as HTMLInputElement;
          button.addEventListener('click', () => {
            const value = parseFloat(input.value);
            if (!isNaN(value)) {
              ws.send(JSON.stringify({ type: 'link_setpoints', data: [{ link_name: link.link_name, value: value }] }));
              console.log('Sent link_setpoints:', { link_name: link.link_name, value: value });
            }
          });
        }
      });
      controlsInitialized = true; // Set flag to true after creating controls
    }

    // Clear previous meshes
    meshes.forEach(mesh => scene.remove(mesh));
    meshes.length = 0;

    // Create meshes for each link
    links.forEach((link, index) => {
      // Determine geometry based on non-zero translation
      let geometry: THREE.BoxGeometry;
      let length: number;
      let translateX = 0, translateY = 0, translateZ = 0;

      if (link.translation.x !== 0) {
        length = link.translation.x;
        geometry = new THREE.BoxGeometry(length, 0.2, 0.2); // Extend in X
        translateX = length / 2; // Origin at X=0
      } else if (link.translation.y !== 0) {
        length = link.translation.y;
        geometry = new THREE.BoxGeometry(0.2, length, 0.2); // Extend in Y
        translateY = length / 2; // Origin at Y=0
      } else {
        length = link.translation.z || 0.75; // Default 0.75m if zero
        geometry = new THREE.BoxGeometry(0.2, 0.2, length); // Extend in Z
        translateZ = length / 2; // Origin at Z=0
      }

      geometry.translate(translateX, translateY, translateZ); // Set origin at extension start

      // Material and color
      const material = new THREE.MeshBasicMaterial({
        color: link.movable === 'STATIC' ? 0x95a5a6  :
               link.link_name === 'base' ? 0xd35400  :
               link.link_name === 'arm1' ? 0x0000ff : 
               link.link_name === 'arm2' ? 0xff0000 :
               link.link_name === 'arm3' ? 0xffff00 :              
               0x00ff00
      });

      const mesh = new THREE.Mesh(geometry, material);

      // Apply rotation first (in radians)
      mesh.rotation.set(
        link.rotation.x,
        link.rotation.y,
        link.rotation.z
      );

      // Parent to previous mesh (hierarchical)
      if (meshes.length > 0) {
              // Apply translation in rotated frame
        mesh.position.set(links[index-1].translation.x, links[index-1].translation.y, links[index-1].translation.z);
        meshes[meshes.length - 1].add(mesh);
      } else {
        scene.add(mesh);
      }
      meshes.push(mesh);
    });
  } catch (error) {
    console.error('Invalid JSON:', error);
  }
};

// Send All button handler
const sendAllButton = document.getElementById('send-all') as HTMLButtonElement;
sendAllButton.addEventListener('click', () => {
  const requests = links
    .filter(link => link.movable !== 'STATIC' && link.movable !== 'Move_Base')
    .map(link => {
      const input = document.getElementById(`input-${link.link_name}`) as HTMLInputElement;
      const value = parseFloat(input.value);
      return { link_name: link.link_name, value: isNaN(value) ? 0 : value };
    });
  if (requests.length > 0) {
    ws.send(JSON.stringify({ type: 'link_setpoints', data: requests }));
    console.log('Sent link_setpoints:', requests);
  }
});

// Send Goal button handler
const sendGoalButton = document.getElementById('send-goal') as HTMLButtonElement;
sendGoalButton.addEventListener('click', () => {
  const goalPose: GoalPose = {
    x: parseFloat((document.getElementById('goal-x') as HTMLInputElement).value) || 0,
    y: parseFloat((document.getElementById('goal-y') as HTMLInputElement).value) || 0,
    z: parseFloat((document.getElementById('goal-z') as HTMLInputElement).value) || 0,
    rotz: parseFloat((document.getElementById('goal-rotz') as HTMLInputElement).value) || 0
  };
  ws.send(JSON.stringify({ type: 'goal_setpoints', data: { goal_pose: goalPose } }));
  console.log('Sent goal_setpoints:', goalPose);
});

// Send Move Base button handler
const sendMoveBaseButton = document.getElementById('move_base') as HTMLButtonElement;
sendMoveBaseButton.addEventListener('click', () => {
  const baseGoalPose: GoalPose = {
    x: parseFloat((document.getElementById('base-x') as HTMLInputElement).value) || 0,
    y: parseFloat((document.getElementById('base-y') as HTMLInputElement).value) || 0,
    z: parseFloat((document.getElementById('base-z') as HTMLInputElement).value) || 0,
    rotz: parseFloat((document.getElementById('base-rotz') as HTMLInputElement).value) || 0
  };
  ws.send(JSON.stringify({ type: 'move_base_setpoints', data: { goal_pose: baseGoalPose } }));
  console.log('Sent goal_setpoints:', baseGoalPose);
});

ws.onerror = (error: Event) => console.error('WebSocket error:', error);
ws.onclose = () => console.log('WebSocket closed');

// Three.js setup
const camera = new THREE.PerspectiveCamera(75, 600 / 400, 0.1, 1000);
camera.position.set(3.5, 2, 2.5);
camera.lookAt(0, 0, 1.5);
camera.rotateZ(THREE.MathUtils.degToRad(97));
// if I want to look at it from above
// camera.position.set(0, 0, 5);
// camera.lookAt(0, 0, 2);

// add ground plane grid
const gridSize = 10; // in meters
const divisions = 10;
const gridHelper = new THREE.GridHelper(gridSize, divisions, 0xe74c3c, 0x888888); // Colors: center line and grid lines
gridHelper.position.set(0, 0, 0); // Place it at z=0 (ground plane)
gridHelper.rotation.x = THREE.MathUtils.degToRad(90); // Not sure why, but needs rotation
scene.add(gridHelper);

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('three-canvas') as HTMLCanvasElement });
renderer.setSize(900, 600);

// Animation loop
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();
