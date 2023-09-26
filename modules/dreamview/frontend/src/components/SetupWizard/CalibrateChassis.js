import React from "react";
import classNames from "classnames";
import STORE from "store";
import WS from "store/websocket";
import Dashboard from 'components/SetupWizard/Dashboard';

export default class CalibrateChassisComponent extends React.Component {
    constructor(props) {
        super(props);
        this.onKeyEvent = this.onKeyEvent.bind(this);
        this.onButtonClicked = this.onButtonClicked.bind(this);
        this.state = {
            throttle: 0.0,
            brake: 0.0,
            steer: 0.0,
        };
        this.throttle_timestamp = Date.now();
        this.steering_timestamp = Date.now();
    }
    componentDidMount() {
        document.addEventListener("keydown", this.onKeyEvent);
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            const curtime = Date.now();
            if (curtime - this.throttle_timestamp > 500) {
                let xthrottle = this.state.throttle - this.state.throttle / 2;
                if (xthrottle < 10) {
                    xthrottle = 0;
                }
                this.throttle_timestamp = Date.now();
                this.setState({ throttle: xthrottle });
            }
            if (curtime - this.steering_timestamp > 500) {
                let xsteering = this.state.steer - this.state.steer / 2;
                if (xsteering < 0 && xsteering > -10) {
                    xsteering = 0;
                }
                if (xsteering > 0 && xsteering < 10) {
                    xsteering = 0;
                }
                this.steering_timestamp = Date.now();
                this.setState({ steer: xsteering });
            }
            if (WS.websocket.readyState === WS.websocket.OPEN) {
                const value = this.state;
                WS.websocket.send(JSON.stringify({
                    type: "ControlVerify",
                    value
                }));
            }
        }, 50);
    }
    componentWillUnmount() {
        clearInterval(this.timer);
        this.setState({ steer: 0.0, throttle: 0.0, brake: 0.0 });
        if (WS.websocket.readyState === WS.websocket.OPEN) {
            const value = this.state;
            WS.websocket.send(JSON.stringify({
                type: "ControlVerify",
                value
            }));
        }
        document.removeEventListener('keydown', this.onKeyEvent);
    }
    onButtonClicked() {
        //const value = this.state;
        //WS.websocket.send(JSON.stringify({
        //            type : "ControlVerify",
        //            value}));
    }
    onKeyEvent(evt) {
        console.log(evt);
        let xsteering = 0.0;
        let xthrottle = 0.0;
        switch (evt.which) {
            case 37://Left
                xsteering = this.state.steer + 2;
                if (xsteering > 100) {
                    xsteering = 100;
                }
                this.setState({ steer: xsteering });
                this.steering_timestamp = Date.now();
                this.onButtonClicked();
                break;

            case 38://Up
                xthrottle = this.state.throttle + 2;
                if (xthrottle > 100) {
                    xthrottle = 100;
                }
                this.setState({ throttle: xthrottle });
                this.throttle_timestamp = Date.now();
                this.onButtonClicked();
                break;

            case 39://Right
                xsteering = this.state.steer - 2;
                if (xsteering < -100) {
                    xsteering = -100;
                }
                this.setState({ steer: xsteering });
                this.steering_timestamp = Date.now();
                this.onButtonClicked();
                break;

            case 40://Down
                xthrottle = this.state.throttle - 2;
                if (xthrottle < 0) {
                    xthrottle = 0;
                }
                this.setState({ throttle: xthrottle });
                this.throttle_timestamp = Date.now();
                this.onButtonClicked();
                break;

            case 20://Space
                console.log("[Dashboard] Space");
                this.setState({ throttle: 0.0, brake: 100.0 });
                this.onButtonClicked();
                this.throttle_timestamp = Date.now();
                this.steering_timestamp = Date.now();
                break;
        }
    }
    render() {
        //            renderer: 'canvas',
        const renderer = 'canvas';//this.state.renderer;
        //<Dashboard renderer={renderer} option={option} />
        const option = this.getOption();
        const { handleNextButton } = this.props;
        return (
            <div className="page setup-chassis-page">
                <div className="page-header">
                    <span>底盘验证</span>
                </div>
                <div className="page-content">
                    <table>
                        <tbody>
                            <tr >
                                <td>速度</td>
                                <td>
                                    <span>
                                        <input type="text" value={(this.state.throttle)} name="speed_value" />
                                    </span>
                                </td>
                                <td>转向</td>
                                <td>
                                    <span>
                                        <input type="text" value={(this.state.steer)} name="steering_value" />
                                    </span>
                                </td>
                                <td>
                                    <span>
                                        <button onClick={handleNextButton} className="submit-button">
                                            <span>下一步</span>
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
    getOption = () => {
        return {
            tooltip: {
                formatter: "{a} <br/>{c} {b}"
            },
            toolbox: {
                show: true,
                feature: {
                    restore: { show: true },
                    saveAsImage: { show: true }
                }
            },
            series: [
                {
                    name: '速度',
                    type: 'gauge',
                    center: ['35%', '40%'],
                    z: 3,
                    min: 0,
                    max: 10,
                    splitNumber: 10,
                    radius: '80%',
                    axisLine: {            // 坐标轴线
                        lineStyle: {       // 属性lineStyle控制线条样式
                            width: 6
                        }
                    },
                    axisTick: {            // 坐标轴小标记
                        length: 15,        // 属性length控制线长
                        lineStyle: {       // 属性lineStyle控制线条样式
                            color: 'auto'
                        }
                    },
                    splitLine: {           // 分隔线
                        length: 20,        // 属性length控制线长
                        lineStyle: {       // 属性lineStyle（详见lineStyle）控制线条样式
                            color: 'auto'
                        }
                    },
                    pointer: {
                        width: 4
                    },
                    title: {
                        offsetCenter: ['5%', '45%'],
                        textStyle: {       // 其余属性默认使用全局文本样式，详见TEXTSTYLE
                            fontWeight: 'normal',
                            fontSize: 16,
                            // fontStyle: 'italic'
                        }
                    },
                    detail: {
                        textStyle: {       // 其余属性默认使用全局文本样式，详见TEXTSTYLE
                            fontWeight: 'normal'
                        }
                    },
                    data: [{ value: (this.state.throttle * 1.5 / 100 * 3600 / 1000).toPrecision(3), name: '速度(km/h)' }]
                },
                {
                    name: '转向',
                    type: 'gauge',
                    center: ['65%', '45%'],    // 默认全局居中
                    radius: '90%',
                    min: 0,
                    max: 2,
                    startAngle: 180,
                    endAngle: 0,
                    splitNumber: 20,
                    axisLine: {            // 坐标轴线
                        lineStyle: {       // 属性lineStyle控制线条样式
                            width: 4
                        }
                    },
                    axisTick: {            // 坐标轴小标记
                        splitNumber: 5,
                        length: 10,        // 属性length控制线长
                        lineStyle: {       // 属性lineStyle控制线条样式
                            color: 'auto'
                        }
                    },
                    axisLabel: {
                        formatter: function (v) {
                            switch (v + '') {
                                case '0': return '100%';
                                case '1': return '0%';
                                case '2': return '100%';
                            }
                        }
                    },
                    splitLine: {           // 分隔线
                        length: 15,        // 属性length控制线长
                        lineStyle: {       // 属性lineStyle（详见lineStyle）控制线条样式
                            color: 'auto'
                        }
                    },
                    pointer: {
                        width: 2
                    },
                    title: {
                        offsetCenter: ['5%', '35%'],       // x, y，单位px
                    },
                    detail: {
                        show: false
                    },
                    data: [{ value: 1 + (-this.state.steer / 100), name: '转向率' }]
                }
            ]
        };
    };
}