import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";
import ButtonPanel from "components/SideBar/ButtonPanel";
import SecondaryButton from "components/SideBar/SecondaryButton";
import WS from "store/websocket";
import powerOffIcon from "assets/images/sidebar/poweroff.png";

import intl from 'react-intl-universal';


class PowerOffButton extends React.PureComponent {
    render() {
        const { disabled, onClick, active, label, extraClasses, iconSrc } = this.props;
        return (
            <button onClick={onClick} disabled={disabled}
                    className={classNames({
                            "button": true,
                            "button-active": active,
                        }, extraClasses)} >
                <img src={iconSrc} className="icon" />
                <div className="label">{label}</div>
            </button>
        );
    }
}

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly, hmi } = this.props.store;

        const settings = {};
        const optionNames = [...options.mainSideBarOptions, ...options.secondarySideBarOptions];
        optionNames.forEach(optionName => {
            settings[optionName] = {
                active: options[optionName],
                onClick: () => {
                    this.props.store.handleOptionToggle(optionName);
                },
                disabled: options.isSideBarButtonDisabled(
                    optionName,
                    enableHMIButtonsOnly,
                    hmi.inNavigationMode
                ),
            };
        });

        return (
            <div className="side-bar">
                <ButtonPanel settings={settings} />
                <div className="sub-button-panel">
                    <SecondaryButton
                        panelLabel={intl.get('Audio_Capture')}
                        disabled={settings.enableAudioCapture.disabled}
                        onClick={settings.enableAudioCapture.onClick}
                        active={settings.enableAudioCapture.active} />
                    <SecondaryButton
                        panelLabel={intl.get('Default_Routing')}
                        disabled={settings.showPOI.disabled}
                        onClick={settings.showPOI.onClick}
                        active={!options.showRouteEditingBar && options.showPOI} />
                    <PowerOffButton
                        label={intl.get('Power_Off')}
                        disabled={!hmi.enablePowerOff}
                        iconSrc={powerOffIcon}
                        onClick={() => this.props.store.hmi.togglePowerOff()}
                        active={hmi.enablePowerOff} />
                </div>
            </div>
        );
    }
}
