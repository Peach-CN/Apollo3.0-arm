import React from "react";
import PropTypes from 'prop-types';
import { observer } from "mobx-react";
import WS from "store/websocket";
import AutoMeter from "components/StatusBar/AutoMeter";
import Notification from "components/StatusBar/Notification";
import TrafficLightIndicator from "components/StatusBar/TrafficLightIndicator";
import DrivingMode from "components/StatusBar/DrivingMode";
import Wheel from "components/StatusBar/Wheel";
import MapRecord from "components/StatusBar/MapRecord";
import intl from 'react-intl-universal';
class Battery extends React.PureComponent {
	  static propTypes = {
    	percent: PropTypes.number.isRequired,
    	voltage: PropTypes.number.isRequired,
    	color: PropTypes.string,
    	gridColor: PropTypes.string,
    	size: PropTypes.string,
    	type: PropTypes.oneOf(['default', 'nogrid'])
  	}

  	static defaultProps = {
    	percent: 100,
    	voltage: 29.0,
    	color: 'green',
    	gridColor: '#fff',
    	size: '50',
    	type: 'nogrid',
  	}

  	render() {
    	const { percent, voltage, color, gridColor, size, type } = this.props;
    	let offset = 0;
    	let fcolor = color;

    	if (percent % 20 === 0 && percent !== 0) {
      		offset = -50;
    	}
    	if (percent < 30) {
      		fcolor = 'yellow';
    	}
    	if (percent < 15) {
      		fcolor = 'red';
    	}

    	return (
      		<div className="battery-hint">
        		<div className="battery-chart">
          			<svg width='116px' height='35px' viewBox='0 0 1024 1024'>
            			<rect x={140} y={350} width={type !== 'nogrid' ? 500 * percent / 100 + Math.floor(percent / 20) * 50 + offset : 700 * percent / 100} height={330} style={{ fill: fcolor }} />
            			{type !== 'nogrid' && new Array(5).fill(0).map((item, index) =>
              			<rect key={index} x={240 + index * 150} y={340} width={50} height={350} style={{ fill: gridColor }} />)}
            			<path style={{ fill: fcolor }} d="M938.666667 426.666667l0-85.333333c0-46.933333-38.4-85.333333-85.333333-85.333333L128 256c-46.933333 0-85.333333 38.4-85.333333 85.333333l0 341.333333c0 46.933333 38.4 85.333333 85.333333 85.333333l725.333333 0c46.933333 0 85.333333-38.4 85.333333-85.333333l0-85.333333c23.466667 0 42.666667-19.2 42.666667-42.666667l0-85.333333C981.333333 445.866667 962.133333 426.666667 938.666667 426.666667zM896 469.333333l0 85.333333 0 128c0 23.466667-19.2 42.666667-42.666667 42.666667L128 725.333333c-23.466667 0-42.666667-19.2-42.666667-42.666667L85.333333 341.333333c0-23.466667 19.2-42.666667 42.666667-42.666667l725.333333 0c23.466667 0 42.666667 19.2 42.666667 42.666667L896 469.333333z" />
          			</svg>
        		</div>
        		<div className="battery-text">
          			<div className="text">{voltage}V / {percent}%</div>
        		</div>
      		</div>
    	);
  	}
}

@observer
export default class StatusBar extends React.Component {
  	render() {
    	const { meters, trafficSignal, showNotification, monitor } = this.props;

    	return (
      		<div className="status-bar">
        		{showNotification && <Notification monitor={monitor} />}
        		<AutoMeter throttlePercent={meters.throttlePercent}
          				brakePercent={meters.brakePercent}
          				speed={meters.speed} />
        		<Wheel steeringPercentage={meters.steeringPercentage}
          				steeringAngle={meters.steeringAngle}
          				turnSignal={meters.turnSignal} />
        		<div className="traffic-light-and-driving-mode">
          			<TrafficLightIndicator colorName={trafficSignal.color} />
          			<DrivingMode drivingMode={meters.drivingMode}
            				isAutoMode={meters.isAutoMode} />
        		</div>
        		<Battery percent={meters.batteryCapacity}
          				voltage={meters.batteryVoltage} />
        		<MapRecord />
      		</div>
        );
    }
}
