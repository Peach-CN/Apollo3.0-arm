import React from "react";
import classNames from "classnames";
import STORE from "store";
import ReactTooltip from 'react-tooltip';
import configIcon from "assets/images/icons/config.png";
import WS from "store/websocket";
import helpIcon from "assets/images/icons/help.png";

export default class SetupVehicleComponent extends React.Component {
	constructor(props) {
		super(props);
        this.onValueChanged = this.onValueChanged.bind(this);
        this.onButtonClicked = this.onButtonClicked.bind(this);
        const config = STORE.hmi.vehicleParam;
        this.state = {
            front_edge_to_center: String(config.frontEdgeToCenter),
            back_edge_to_center: String(config.backEdgeToCenter),
            left_edge_to_center: String(config.leftEdgeToCenter),
            right_edge_to_center: String(config.rightEdgeToCenter),
            min_turn_radius: String(config.minTurnRadius),
            max_steer_angle: String(config.maxSteerAngle),
            wheel_base: String(config.wheelBase),
            maximum_speed: "1.0",
        };
    }

    onValueChanged(e) {
        this.setState({ [e.target.name]: e.target.value });
    }

    onButtonClicked() {
        const value = this.state;
        WS.websocket.send(JSON.stringify({
            type: "VehicleSettingCommand",
            value,
        }));
        const { handleNextButton } = this.props;
        handleNextButton();
    }

    render() {
        return (
            <div className="page setup-vehicle-page">
                <div className="page-header">
                    <span>车辆配置</span>
                </div>
                <div className="page-content">
                    <table>
                        <tbody>
                            <tr>
                                <td>车辆前沿距离中心距离</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.front_edge_to_center} name="front_edge_to_center"
                                            placeholder={this.state.back_edge_to_center} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="测量车辆前沿距离后轮中心线的距离，单位：m" />
                                        <ReactTooltip place='top' type='light' />
                                    </span>
                                </td>
                                <td>车辆后沿距离中心距离</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.back_edge_to_center} name="back_edge_to_center"
                                            placeholder={this.state.back_edge_to_center} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="测量车辆后沿距离后轮中心线的距离，单位：m" />
                                    </span>
                                </td>
                            </tr>
                            <tr>
                                <td>车辆左侧距离后轮轴中心点距离</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.left_edge_to_center} name="left_edge_to_center"
                                            placeholder={this.state.left_edge_to_center}
                                            onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="测量车辆左侧距离后轮中心垂线的距离，单位：m" />
                                    </span>
                                </td>
                                <td>车辆右侧距离后轮轴中心点距离</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.right_edge_to_center} name="right_edge_to_center"
                                            placeholder={this.state.right_edge_to_center} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="测量车辆右侧距离后轮中心垂线的距离，单位：m" />
                                    </span>
                                </td>
                            </tr>
                            <tr>
                                <td>车辆最小转弯半径</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.min_turn_radius} name="min_turn_radius"
                                            placeholder={this.state.min_turn_radius} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="在阿克曼底盘中，最小转弯半径是指车辆在最大转向角的情况下从Y轴平行方向转向到X轴平行方向行进的弧形半径，单位：m" />
                                    </span>
                                </td>
                                <td>车辆最大转向角</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.max_steer_angle} name="max_steer_angle"
                                            placeholder={this.state.max_steer_angle} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="在阿克曼底盘中，最大转向角是指前轮向左或向右最大转多少度，单位：rad（弧度）；例如43°=43/180xπ" />
                                    </span>
                                </td>
                            </tr>
                            <tr>
                                <td>车辆后轮轮距</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.wheel_base} name="wheel_base"
                                            placeholder={this.state.wheel_base} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="两个后轮中心的距离，单位：m" />
                                    </span>
                                </td>
                                <td>车辆最高速度</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.maximum_speed} name="maximum_speed"
                                            placeholder={this.state.maximum_speed} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="车辆最高行驶速度，单位：m/s" />
                                    </span>
                                </td>
                                <td rowSpan="4" align="center">
                                    <span>
                                        <button onClick={this.onButtonClicked} className="common-button submit-button">
                                            <span data-tip="点击按钮保存车辆参数">下一步</span>
                                        </button>
                                    </span>
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>
        );
    }
}