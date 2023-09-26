import React from "react";
import classNames from "classnames";
import STORE from "store";
import ReactTooltip from 'react-tooltip';
import WS from "store/websocket";
import configIcon from "assets/images/icons/config.png";
import helpIcon from "assets/images/icons/help.png";

export default class SetupGPSComponent extends React.Component {
    constructor(props) {
        super(props);
        this.onValueChanged = this.onValueChanged.bind(this);
        this.onButtonClicked = this.onButtonClicked.bind(this);
        const config = STORE.hmi.gnssParam;
        this.state = {
            rtk_base_addr: config.rtkBaseAddr,
            rtk_base_port: config.rtkBasePort,
            rtk_base_username: config.rtkBaseUsername,
            rtk_base_passwd: config.rtkBasePasswd,
            rtk_base_mountpoint: config.rtkBaseMountpoint,
            local_addr: config.localAddr,
            local_netmask: config.localNetmask,
            local_gateway: config.localGateway,
            head_offset: config.headOffset,
            leverarm1_x: config.leverarm1X,
            leverarm1_y: config.leverarm1Y,
            leverarm1_z: config.leverarm1Z,
            leverarm2_x: config.leverarm2X,
            leverarm2_y: config.leverarm2Y,
            leverarm2_z: config.leverarm2Z,
        };
    }
    onValueChanged(e) {
        this.setState({ [e.target.name]: e.target.value });
    }
    onButtonClicked() {
        const value = this.state;
        WS.websocket.send(JSON.stringify({
            type: "GnssSettingCommand",
            value,
        }));
        const { handleNextButton } = this.props;
        handleNextButton();
    }

    render() {
        return (
            <div className="page setup-gnss-page">
                <div className="page-header">
                    <span>GPS接收机配置</span>
                </div>
                <div className="page-content">
                    <table>
                        <tbody>
                            <tr >
                                <td>RTK基站地址</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.rtk_base_addr} name="rtk_base_addr"
                                            placeholder={this.state.rtk_base_addr} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="填写自建RTK基站或差分数据提供商的IP地址" />
                                        <ReactTooltip place='top' type='light' />
                                    </span>
                                </td>
                                <td>RTK基站端口</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.rtk_base_port} name="rtk_base_port"
                                            placeholder={this.state.rtk_base_port} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="填写自建RTK基站或差分数据提供商的端口号" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>RTK基站用户名</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.rtk_base_username} name="rtk_base_username"
                                            placeholder={this.state.rtk_base_username} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="填写登录RTK基站的用户名（可咨询定位服务提供商获得）" />
                                    </span>
                                </td>
                                <td>RTK基站密码</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.rtk_base_passwd} name="rtk_base_passwd"
                                            placeholder={this.state.rtk_base_passwd} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="填写登录RTK基站的密码（可咨询定位服务提供商获得）" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>RTK基站挂载点</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.rtk_base_mountpoint} name="rtk_base_mountpoint"
                                            placeholder={this.state.rtk_base_mountpoint} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="RTK基站的挂载点（可咨询定位服务提供商获得）" />
                                    </span>
                                </td>
                                <td>HeadOffset</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.head_offset} name="head_offset"
                                            placeholder={this.state.head_offset} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="根据GPS天线和接收机的位置关系填写，单位为度（可参考接收机配置手册）" />
                                    </span>
                                </td>

                            </tr>
                            <tr >
                                <td>接收机IP地址</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.local_addr} name="local_addr"
                                            placeholder={this.state.local_addr} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="为接收机分配固定IP地址（填写4G路由器分配的IP网段中未被占用的IP地址）" />
                                    </span>
                                </td>
                                <td>接收机IP子网掩码</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.local_netmask} name="local_netmask"
                                            placeholder={this.state.local_netmask} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="为接收机分配的固定IP地址的子网掩码（例如：255.255.255.0）" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>接收机IP网关</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.local_gateway} name="local_gateway"
                                            placeholder={this.state.local_gateway} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="为接收机分配的固定IP地址网关（例如：192.168.1.1）" />
                                    </span>
                                </td>
                                <td>主杆臂值（X）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm1_x} name="leverarm1_x"
                                            placeholder={this.state.leverarm1_x} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS主天线与接收机在X方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>主杆臂值（Y）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm1_y} name="leverarm1_y"
                                            placeholder={this.state.leverarm1_y} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS主天线与接收机在Y方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                                <td>主杆臂值（Z）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm1_z} name="leverarm1_z"
                                            placeholder={this.state.leverarm1_z} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS主天线与接收机在Z方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>副杆臂值（X）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm2_x} name="leverarm2_x"
                                            placeholder={this.state.leverarm2_x} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS副天线与接收机在X方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                                <td>副杆臂值（Y）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm2_y} name="leverarm2_y"
                                            placeholder={this.state.leverarm2_y} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS副天线与接收机在Y方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                            </tr>
                            <tr >
                                <td>副杆臂值（Z）</td>
                                <td>
                                    <span>
                                        <input type="text" defaultValue={this.state.leverarm2_z} name="leverarm2_z"
                                            placeholder={this.state.leverarm2_z} onChange={this.onValueChanged} />
                                        <img src={helpIcon} data-tip="手动测量GPS副天线与接收机在Z方向上的距离，单位为米（可参考接收机配置手册）" />
                                    </span>
                                </td>
                                <td rowSpan="6" align="right">
                                    <span>
                                        <button onClick={this.onButtonClicked} className="submit-button">
                                            <span data-tip="点击按钮就配置信息下发到接收机">下一步</span>
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
