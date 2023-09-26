import STORE from "store";
import Worker from 'utils/webworker.js';

export default class SystemControlWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.worker = new Worker();
    }

    initialize() {
        try {
            this.websocket = new WebSocket(this.serverAddr);
            this.websocket.binaryType = "arraybuffer";
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onmessage = event => {
            this.worker.postMessage({
                source: 'systemctl',
                data: event.data,
            });
        };
        this.worker.onmessage = event => {
			console.log("[SystemControlWebSocketEndpoint] this.worker.onmessage = event =>" + event.data);
            //this.currentMode = STORE.hmi.currentMode;
            //RENDERER.updateMap(event.data, removeOldMap);
            //STORE.setInitializationStatus(true);
        };
        this.websocket.onclose = event => {
			STORE.hmi.enablePowerOff = false;
            console.log("WebSocket connection closed with code: " + event.code);
            this.initialize();
        };
		STORE.hmi.enablePowerOff = true;
    }

    execSystemCmd(evt) {
        this.websocket.send(JSON.stringify({
            type: "SystemControl",
            command: evt,
        }));
    }
}
