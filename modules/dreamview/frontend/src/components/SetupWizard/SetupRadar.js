import React from "react";
import classNames from "classnames";
import STORE from "store";
import ReactTooltip from 'react-tooltip';
import WS from "store/websocket";
import configIcon from "assets/images/icons/config.png";
import helpIcon from "assets/images/icons/help.png";

export default class SetupRadarComponent extends React.Component {
    constructor(props) {
        super(props);
        this.onButtonClicked = this.onButtonClicked.bind(this);
        const config = STORE.senior.radarParam;
        this.state = {
            //当前传感器实时数据,数组
            seniorCurrentValue: config.senior_realtimeValue,
            //传感器数量
            seniorNumber: config.senior_realtimeValue.length,
            //传感器障碍物探测距离
            seniorDetectValue: config.radarDetectObjectParam,
        };
    }
    //组件挂后
    componentDidMount() {
        this.timerID = setInterval(
            () => {
              this.setState({
                seniorCurrentValue: STORE.senior.radarParam.senior_realtimeValue
              });
            },
            50);
    }
    //组件卸载前
    componentWillUnmount() {
        clearInterval(this.timerID);
    }
    //点击下一步按钮事件
    onButtonClicked() {
        WS.websocket.send(JSON.stringify({
            type: "RadarDetectSetting",
            data: this.state.seniorDetectValue
        }));
        console.log(this.state.seniorDetectValue);
        const { handleNextButton } = this.props;
        handleNextButton();
    }
    render() {
        return (
            <div className="page setup-radar-page">
                <div className="page-header">
                    <span>超声波雷达</span>
                </div>
                <div className="page-content">
                    <span>请先打开"仿真控制器"和"超声波模块"</span>
                    <table>
                        <tbody>
                            {
                                this.state.seniorDetectValue.config.map( (item,index) => {
                                    return(
                                        <tr key={index}>
                                            <td>探头{item.direction}实时值:</td>
                                            <td>
                                                <p>{this.state.seniorCurrentValue[item.index]}</p>
                                            </td>
                                            <td>探头{item.direction}危险预警值:</td>
                                            <td>
                                                <input type="number"
                                                    step="0.01"
                                                    placeholder={item.distance}
                                                    defaultValue={item.distance}
                                                    onChange={(e) => {
                                                        this.state.seniorDetectValue.config[index].distance=parseFloat(e.target.value);
                                                    }} />
                                            </td>
                                        </tr>
                                    );
                                })
                            }
                            <tr>
                                <td colSpan="4" align="right">
                                    <span>
                                        <button className="submit-button" onClick={this.onButtonClicked}>
                                            <span data-tip="点击按钮完成信息配置">下一步</span>
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