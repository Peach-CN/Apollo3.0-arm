import { observable, action, computed } from "mobx";

import WS, {SYSTEM_CTL_WS} from "store/websocket";

export default class Senior {
    radarParam = {
        senior_realtimeValue: [],//超声波传感器实时数据
        radarDetectObjectParam:{} //超声波检测障碍物阈值
    };
    updateRadarRealTimeParam(seniorParam) {
        if (seniorParam) {
            this.radarParam.senior_realtimeValue = seniorParam;
        }
    }
    updateRadarDetectObjectParam(radarDetectObject) {
        if (radarDetectObject) {
            this.radarParam.radarDetectObjectParam = radarDetectObject;
        }
    }
}
