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
  } catch (error) {
    console.error('Invalid JSON:', error);
  }
};

ws.onerror = (error: Event) => console.error('WebSocket error:', error);
ws.onclose = () => console.log('WebSocket closed');