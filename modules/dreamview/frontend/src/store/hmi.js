import { observable, action, computed } from "mobx";

import WS, {SYSTEM_CTL_WS} from "store/websocket";

export default class HMI {

    modes = {};
    @observable currentMode = 'none';

    vehicles = [];
    @observable currentVehicle = 'none';
    vehicleParam = {
        frontEdgeToCenter: 3.89,
        backEdgeToCenter: 1.04,
        leftEdgeToCenter: 1.055,
        rightEdgeToCenter: 1.055,
        height: 1.48,
        width: 2.11,
        length: 4.933,
        steerRatio: 16,
        minTurnRadius: 5.05386147161,
        maxAcceleration: 2.0,
        maxDeceleration: -6.0,
        maxSteerAngle: 8.20304748437,
        maxSteerAngleRate: 6.98131700798,
	      minSteerAngleRate: 0,
        wheelBase: 2.8448,
        wheelRollingRadius: 0.335,
    };
    gnssParam = {
        rtkBaseAddr: "",
        rtkBasePort: "",
        rtkBaseUsername: "",
        rtkBasePasswd: "",
        rtkBaseMountpoint: "",
        localAddr: "",
        localNetmask: "",
        localGateway: "",
        headOffset: "",
        leverarm1X: "",
        leverarm1Y: "",
        leverarm1Z: "",
        leverarm2X: "",
        leverarm2Y: "",
        leverarm2Z: "",
    };
    maps = [];
    @observable currentMap = 'none';

    @observable moduleStatus = observable.map();
    @observable hardwareStatus = observable.map();
    @observable enableStartAuto = false;

    displayName = {};
    utmZoneId = 10;
    utterance = window.speechSynthesis ? new SpeechSynthesisUtterance() : null;

    @observable dockerImage = 'unknown';

    @observable isCoDriver = false;
    @observable showSetupWizard = false;
    @observable isDisinfectionMode = false;
    @observable enablePowerOff = false;

    @action initialize(config) {
        if (config.dockerImage) {
            this.dockerImage = config.dockerImage;
        }
        if (config.modes) {
            this.modes = config.modes;
        }
        if (config.utmZoneId) {
            this.utmZoneId = config.utmZoneId;
        }
        this.vehicles = Object.keys(config.availableVehicles).sort()
            .map(name => {
                return name;
            });
        this.maps = Object.keys(config.availableMaps).sort()
            .map(name => {
                return name;
            });

        Object.keys(config.modules).forEach(key => {
            this.moduleStatus.set(key, false);
            this.displayName[key] = config.modules[key].displayName;
        });
        Object.keys(config.hardware).forEach(key => {
            this.hardwareStatus.set(key, 'NOT_READY');
            this.displayName[key] = config.hardware[key].displayName;
        });
    }

    @action toggleCoDriverFlag() {
        this.isCoDriver = !this.isCoDriver;
    }

    @action toggleSetupWizard() {
        this.showSetupWizard = !this.showSetupWizard;
    }
    @action toggleDisinfection() {
        this.isDisinfectionMode = !this.isDisinfectionMode;
            WS.websocket.send(JSON.stringify({
                type : "SwitchOnTheValveOfSprayer",
                command : this.isDisinfectionMode ? "On" : "Off",
        }));
    }
    @action togglePowerOff() {
        SYSTEM_CTL_WS.execSystemCmd("shutdown");
    }
    @action toggleReboot() {
        SYSTEM_CTL_WS.execSystemCmd("reboot");
    }
    @action updateStatus(newStatus) {
        if (newStatus.currentMode) {
            this.currentMode = newStatus.currentMode;
        }
        if (newStatus.currentMap) {
            this.currentMap = newStatus.currentMap;
        }
        if (newStatus.currentVehicle) {
            this.currentVehicle = newStatus.currentVehicle;
        }
        if (newStatus.systemStatus) {
            if (newStatus.systemStatus.modules) {
                for (const key in newStatus.systemStatus.modules) {
                    this.moduleStatus.set(key,
                        newStatus.systemStatus.modules[key].processStatus.running);
                }
            }
            if (newStatus.systemStatus.hardware) {
                for (const key in newStatus.systemStatus.hardware) {
                    this.hardwareStatus.set(key, newStatus.systemStatus.hardware[key].summary);
                }
            }
            if (this.utterance &&
                typeof newStatus.systemStatus.passengerMsg === "string" &&
                newStatus.systemStatus.passengerMsg !== this.utterance.text) {
                    this.utterance.text = newStatus.systemStatus.passengerMsg;
                this.speakPassengerMessage();
            }
        }
    }

    speakPassengerMessage() {
        if (this.utterance.text) {
            // if speaking, don't interrupt
            if (!window.speechSynthesis.speaking) {
                window.speechSynthesis.speak(this.utterance);
            }

            // repeat this message until a new one is given
            this.utterance.onend = () => {
                window.speechSynthesis.speak(this.utterance);
            };
        } else {
            this.utterance.onend = null;
        }
    }

    @action update(world) {
        this.enableStartAuto = world.engageAdvice === "READY_TO_ENGAGE";
    }

 //   DriveModechanged(mode) {
 //       this.enableStartAuto = mode === "COMPLETE_AUTO_DRIVE";
 //   }

    updateVehicleParam(vehicleParam) {
        this.vehicleParam = vehicleParam;
    }

    updateGnssParam(gnssParam) {
        this.gnssParam = gnssParam;
    }

    @action toggleModule(id) {
        this.moduleStatus.set(id, !this.moduleStatus.get(id));
        const command = this.moduleStatus.get(id) ? "start" : "stop";
        WS.executeModuleCommand(id, command);
    }

    @computed get showRTKCommands() {
        return this.currentMode === "RTK Record / Replay";
    }

    @computed get inNavigationMode() {
        return this.currentMode === "Navigation";
    }
}
