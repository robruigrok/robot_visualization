import * as THREE from 'three';

interface LinkState {
  link_name: string;
  movable: string;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number };
}

const ws = new WebSocket('ws://localhost:3000');
const stateDiv = document.getElementById('state')!;
const controlsDiv = document.getElementById('controls')!;
const scene = new THREE.Scene();
const meshes: THREE.Mesh[] = []; // Store link meshes

// WebSocket handlers
ws.onopen = () => console.log('Connected to WebSocket server');

ws.onmessage = (event: MessageEvent) => {
  try {
    const links: LinkState[] = JSON.parse(event.data);
    console.log('Received:', links);

    // Clear previous UI
    stateDiv.innerHTML = '';
    controlsDiv.innerHTML = '';

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
          stateText = `${link.translation.x.toFixed(1)} m`;
          break;
        case 'Y':
          stateText = `${link.translation.y.toFixed(1)} m`;
          break;
        case 'Z':
          stateText = `${link.translation.z.toFixed(1)} m`;
          break;
        case 'ROT_X':
          stateText = `${link.rotation.x.toFixed(1)} deg`;
          break;
        case 'ROT_Y':
          stateText = `${link.rotation.y.toFixed(1)} deg`;
          break;
        case 'ROT_Z':
          stateText = `${link.rotation.z.toFixed(1)} deg`;
          break;
        default:
          stateText = 'Unknown';
      }
      stateP.innerHTML = `${link.link_name}: <span id="state-${link.link_name}">${stateText}</span>`;
      stateDiv.appendChild(stateP);

      // Controls for movable links
      if (link.movable !== 'STATIC') {
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
            const radians = THREE.MathUtils.degToRad(value);
            ws.send(JSON.stringify({ link: index, value: radians }));
            console.log('Sent:', { link: index, value: radians });
            // input.value = ''; // Optional: Clear input
          }
        });
      }
    });

    // Clear previous meshes
    meshes.forEach(mesh => scene.remove(mesh));
    meshes.length = 0;

    // Create meshes for each link
    links.forEach(link => {
      let geometry: THREE.BoxGeometry;
      if (link.link_name === 'base') {
        geometry = new THREE.BoxGeometry(0.3, 0.3, 1.5);
        geometry.translate(0, 0, 0.75); // Center at bottom
      } else {
        const length = link.translation.x; // Use translation.x as arm length
        geometry = new THREE.BoxGeometry(length, 0.2, 0.2);
        geometry.translate(length / 2, 0, 0); // Pivot at end
      }

      const material = new THREE.MeshBasicMaterial({ color: link.link_name === 'base' ? 0x888888 : link.link_name === 'arm1' ? 0xff0000 : 0x00ff00 });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(link.translation.x, link.translation.y, link.translation.z);
      mesh.rotation.set(link.rotation.x, link.rotation.y, link.rotation.z);

      // Parent to previous mesh (hierarchical)
      if (meshes.length > 0) {
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

ws.onerror = (error: Event) => console.error('WebSocket error:', error);
ws.onclose = () => console.log('WebSocket closed');

// Three.js setup
const camera = new THREE.PerspectiveCamera(75, 600 / 400, 0.1, 1000);
camera.position.set(3, 1, 2);
camera.lookAt(0, 0, 2);
camera.rotateZ(THREE.MathUtils.degToRad(90));

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('three-canvas') as HTMLCanvasElement });
renderer.setSize(600, 400);

// Animation loop
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();
