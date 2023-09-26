import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";
import ReactTooltip from 'react-tooltip';
import rebootIcon from "assets/images/sidebar/reboot.png";
import HMISelectors from "components/Header/HMISelectors";
import Selector from "components/Header/Selector";

import intl from 'react-intl-universal';

class PowerOffButton extends React.PureComponent {
    render() {
        const { disabled, onClick, active, label, extraClasses, iconSrc } = this.props;
        return (
            <button onClick={onClick}
                disabled={disabled}
                className={classNames({
                    "button": true,
                }, extraClasses)} >
                <img src={iconSrc} className="icon" data-tip="reboot" />
                <ReactTooltip place='bottom' type='light' />
            </button>
        );
    }
}

@inject("store") @observer
export default class HMIControls extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            language: localStorage.getItem("defaultLng")   // 默认选项
        };
    }

    render() {
        const {
            dockerImage,
            modes, currentMode,
            maps, currentMap,
            vehicles, currentVehicle,
            isCoDriver, showSetupWizard,
            isDisinfectionMode, enablePowerOff
        } = this.props.store.hmi;

        return (
            <React.Fragment>
                <div className="header-item selector">
                    <select value={ this.state.language }
                            onChange={(event) => {
                                console.log(event.target.value);
                                this.setState({
                                    //默认值改变
                                    language:event.target.value
                                });
                                localStorage.setItem('defaultLng', event.target.value);
                                window.location.reload();
                            }}>
                        <option value="zh">中文</option>
                        <option value="en">English</option>
                    </select>
                </div>
                <button className={classNames({"header-item": true,
												"header-button": true,
												"header-button-active": showSetupWizard,
												})}
					onClick={() => this.props.store.hmi.toggleSetupWizard()}>
					{intl.get('Setup_Wizard')}
                </button>
                <button className="header-item header-button"
					onClick={() => alert(dockerImage)}>
                    {intl.get('Check_Version')}
                </button>
                <button className={classNames({"header-item":true,
												"header-button":true,
												"header-button-active": isCoDriver,
												})}
                    onClick={() => this.props.store.hmi.toggleCoDriverFlag()}>
                    {intl.get('Co_Driver')}
                </button>
                <HMISelectors
                    modes={modes}
                    currentMode={currentMode}
                    maps={maps}
                    currentMap={currentMap}
                    vehicles={vehicles}
                    currentVehicle={currentVehicle} />
                <PowerOffButton
                    label="Power Off"
                    disabled={!enablePowerOff}
                    active={enablePowerOff}
                    iconSrc={rebootIcon}
                    onClick={() => this.props.store.hmi.toggleReboot()} />
            </React.Fragment>
        );
    }
}
